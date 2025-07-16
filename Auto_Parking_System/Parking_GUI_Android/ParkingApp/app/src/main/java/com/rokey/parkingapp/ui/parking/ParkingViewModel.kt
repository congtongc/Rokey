package com.rokey.parkingapp.ui.parking

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.rokey.parkingapp.R
import com.rokey.parkingapp.network.*
import kotlinx.coroutines.launch
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import android.util.Log
import com.google.gson.Gson

data class CarTypeInfo(
    val displayText: String,
    val iconResId: Int
)

class ParkingViewModel : ViewModel() {
    private val TAG = "ParkingViewModel"

    private val _cameraStream = MutableLiveData<String>()
    val cameraStream: LiveData<String> = _cameraStream
    
    private val _ocrResult = MutableLiveData<OcrResult>()
    val ocrResult: LiveData<OcrResult> = _ocrResult

    private val _parkingLocation = MutableLiveData<String>()
    val parkingLocation: LiveData<String> = _parkingLocation

    private val _currentCarType = MutableLiveData<String>("normal")
    val currentCarType: LiveData<String> = _currentCarType

    private val _cameraStreamError = MutableLiveData<Boolean>(false)
    val cameraStreamError: LiveData<Boolean> = _cameraStreamError

    private val _errorMessage = MutableLiveData<String>()
    val errorMessage: LiveData<String> = _errorMessage

    private val _isStreaming = MutableLiveData<Boolean>()
    val isStreaming: LiveData<Boolean> = _isStreaming

    private var cameraStreamJob: Job? = null
    private val apiClient = ApiClient.getInstance()
    private val gson = Gson()

    fun startCameraStream() {
        if (cameraStreamJob?.isActive == true) {
            return  // 이미 실행 중이면 중복 실행 방지
        }

        _isStreaming.value = true
        cameraStreamJob = viewModelScope.launch {
            try {
                while (isActive) {
                    try {
                        val response = apiClient.getStream()
                        when (response) {
                            is ApiResponse.Success<String> -> {
                                _cameraStream.postValue(response.data)
                                _errorMessage.postValue(null)
                            }
                            is ApiResponse.Error -> {
                                _errorMessage.postValue("카메라 스트림 오류: ${response.message}")
                            }
                        }
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

    fun getCarTypeInfo(carType: String): CarTypeInfo {
        val info = when (carType.lowercase()) {
            "normal" -> CarTypeInfo("일반 차량", R.drawable.ic_car_normal)
            "ev" -> CarTypeInfo("전기 차량", R.drawable.ic_car_ev)
            "disabled" -> CarTypeInfo("장애인 차량", R.drawable.ic_car_disabled)
            else -> CarTypeInfo("일반 차량", R.drawable.ic_car_normal)
        }
        Log.d(TAG, "차량 타입 정보: $carType -> ${info.displayText}")
        return info
    }

    fun processOcrResult(jsonResult: String) {
        try {
            val ocrResult = gson.fromJson(jsonResult, OcrResult::class.java)
            _ocrResult.postValue(ocrResult)
            // OCR 결과가 있으면 주차 위치 할당 요청
            requestParkingLocation(ocrResult.car_plate, ocrResult.type)
        } catch (e: Exception) {
            _errorMessage.postValue("OCR 결과 처리 실패: ${e.message}")
        }
    }

    fun processManualInput(licensePlate: String, carType: String) {
        val ocrResult = OcrResult(
            car_plate = licensePlate,
            type = carType,
            confidence = 100.0
        )
        _ocrResult.postValue(ocrResult)
        // 수동 입력 시에도 주차 위치 할당 요청
        requestParkingLocation(licensePlate, carType)
    }

    private fun requestParkingLocation(licensePlate: String, carType: String) {
        viewModelScope.launch {
            try {
                val request = ParkingRequest(licensePlate, carType)
                when (val response = apiClient.parkVehicle(request)) {
                    is ApiResponse.Success -> {
                        _parkingLocation.value = response.data.location
                    }
                    is ApiResponse.Error -> {
                        _errorMessage.value = response.message
                        _parkingLocation.value = null
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "주차 위치 할당 중 오류 발생", e)
                _errorMessage.value = e.message
                _parkingLocation.value = null
            }
        }
    }

    fun updateOcrResult(result: OcrResult) {
        _ocrResult.postValue(result)
        _currentCarType.postValue(result.type.lowercase())
        Log.d(TAG, "OCR 결과 업데이트: ${result.car_plate}, 타입: ${result.type}")
        // OCR 결과가 업데이트되면 주차 위치 할당 요청
        result.car_plate.let { plate ->
            parkVehicle(plate, result.type)
        }
    }

    fun updateCarType(type: String) {
        _currentCarType.postValue(type.lowercase())
        Log.d(TAG, "차량 타입 수동 업데이트: $type")
    }

    fun getCurrentCarType(): String {
        return _currentCarType.value ?: "normal"
    }

    fun setCameraStreamError(error: Boolean) {
        _cameraStreamError.value = error
    }

    fun parkVehicle(licensePlate: String, carType: String) {
        viewModelScope.launch {
            try {
                val request = ParkingRequest(licensePlate, carType)
                when (val response = apiClient.parkVehicle(request)) {
                    is ApiResponse.Success -> {
                        _errorMessage.value = "주차가 완료되었습니다"
                    }
                    is ApiResponse.Error -> {
                        _errorMessage.value = response.message
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "주차 처리 중 오류 발생", e)
                _errorMessage.value = e.message
            }
        }
    }

    override fun onCleared() {
        super.onCleared()
        stopCameraStream()
    }
} 