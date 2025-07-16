package com.rokey.parkingapp.ui.parking

import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.rokey.parkingapp.network.*
import kotlinx.coroutines.launch
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import com.google.gson.Gson

class ParkingViewModel : ViewModel() {
    private val TAG = "ParkingViewModel"

    private val _cameraStream = MutableLiveData<String>()
    val cameraStream: LiveData<String> = _cameraStream
    
    private val _ocrResult = MutableLiveData<OcrResult>()
    val ocrResult: LiveData<OcrResult> = _ocrResult
    
    private val _parkingLocation = MutableLiveData<String>()
    val parkingLocation: LiveData<String> = _parkingLocation
    
    private val _errorMessage = MutableLiveData<String>()
    val errorMessage: LiveData<String> = _errorMessage

    private val _isStreaming = MutableLiveData<Boolean>()
    val isStreaming: LiveData<Boolean> = _isStreaming

    private var cameraStreamJob: Job? = null
    private val apiClient = ApiClient.getInstance()
    private val gson = Gson()

    // 주차 위치 관리
    private val parkingLocations = mapOf(
        "normal" to listOf("A-1", "A-2"),
        "ev" to listOf("B-1", "B-2"),
        "disabled" to listOf("C-1", "C-2")
    )

    fun startCameraStream() {
        if (cameraStreamJob?.isActive == true) {
            return  // 이미 실행 중이면 중복 실행 방지
        }

        _isStreaming.value = true
        cameraStreamJob = viewModelScope.launch {
            try {
                while (isActive) {
                    try {
                        val response = apiClient.getCameraFrame()
                        _cameraStream.postValue(response)
                        _errorMessage.postValue(null)
                        delay(100) // 100ms 딜레이로 프레임 레이트 조절
                    } catch (e: Exception) {
                        val errorMsg = when (e) {
                            is java.net.UnknownHostException -> "서버에 연결할 수 없습니다"
                            is java.net.ConnectException -> "서버 연결이 거부되었습니다"
                            is java.net.SocketTimeoutException -> "서버 응답 시간이 초과되었습니다"
                            else -> "카메라 스트림 오류: ${e.message}"
                        }
                        _errorMessage.postValue(errorMsg)
                        delay(1000) // 에러 발생 시 1초 대기 후 재시도
                    }
                }
            } finally {
                _isStreaming.postValue(false)
            }
        }
    }

    fun stopCameraStream() {
        cameraStreamJob?.cancel()
        cameraStreamJob = null
        _isStreaming.postValue(false)
        _errorMessage.postValue(null)
    }

    fun processOcrResult(jsonResult: String) {
        try {
            val ocrResult = gson.fromJson(jsonResult, OcrResult::class.java)
            _ocrResult.postValue(ocrResult)
            if (ocrResult.confidence > 80.0) {
                assignParkingLocation(ocrResult.car_plate, ocrResult.type)
            }
        } catch (e: Exception) {
            _errorMessage.postValue("OCR 결과 처리 실패: ${e.message}")
        }
    }

    private fun assignParkingLocation(licensePlate: String, carType: String) {
        viewModelScope.launch {
            try {
                when (val status = apiClient.getParkingStatus()) {
                    is ApiResponse.Success -> {
                        val locations = parkingLocations[carType] ?: parkingLocations["normal"] ?: listOf()
                        val occupiedLocations = status.data.parkedVehicles?.map { it.location } ?: listOf()
                        
                        val availableLocation = locations.firstOrNull { !occupiedLocations.contains(it) }
                        
                        if (availableLocation != null) {
                            _parkingLocation.postValue(availableLocation)
                            _errorMessage.postValue(null)
                        } else {
                            _errorMessage.postValue("해당 차량 타입의 주차 공간이 없습니다")
                        }
                    }
                    is ApiResponse.Error -> {
                        _errorMessage.postValue(status.message)
                    }
                }
            } catch (e: Exception) {
                _errorMessage.postValue("주차 위치 할당 실패: ${e.message}")
            }
        }
    }

    fun parkVehicle() {
        val currentOcr = _ocrResult.value ?: return
        val location = _parkingLocation.value ?: return

        viewModelScope.launch {
            try {
                val request = ParkingRequest(
                    license_plate = currentOcr.car_plate,
                    car_type = currentOcr.type
                )
                when (val response = apiClient.parkVehicle(request)) {
                    is ApiResponse.Success -> {
                        _errorMessage.postValue("주차가 완료되었습니다")
                        // 주차 완료 후 상태 초기화
                        _ocrResult.postValue(null)
                        _parkingLocation.postValue(null)
                    }
                    is ApiResponse.Error -> {
                        _errorMessage.postValue("주차 실패: ${response.message}")
                    }
                }
            } catch (e: Exception) {
                _errorMessage.postValue("주차 처리 실패: ${e.message}")
            }
        }
    }

    override fun onCleared() {
        super.onCleared()
        stopCameraStream()
    }
} 