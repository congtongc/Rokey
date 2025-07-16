package com.rokey.parkingapp.ui.exit

import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.rokey.parkingapp.network.*
import kotlinx.coroutines.*

class ExitViewModel : ViewModel() {
    
    private val TAG = "ExitViewModel"
    
    private val _parkedVehicles = MutableLiveData<List<ParkedVehicle>>()
    val parkedVehicles: LiveData<List<ParkedVehicle>> = _parkedVehicles
    
    private val _selectedVehicle = MutableLiveData<ParkedVehicle?>()
    val selectedVehicle: LiveData<ParkedVehicle?> = _selectedVehicle
    
    private val _exitResult = MutableLiveData<ExitResponse>()
    val exitResult: LiveData<ExitResponse> = _exitResult
    
    private val _errorMessage = MutableLiveData<String?>()
    val errorMessage: LiveData<String?> = _errorMessage
    
    private val _matchingVehicles = MutableLiveData<List<ParkedVehicle>>()
    val matchingVehicles: LiveData<List<ParkedVehicle>> = _matchingVehicles
    
    private val _cameraStream = MutableLiveData<String>()
    val cameraStream: LiveData<String> = _cameraStream
    
    private val _isStreaming = MutableLiveData<Boolean>()
    val isStreaming: LiveData<Boolean> = _isStreaming
    
    private var cameraStreamJob: Job? = null
    private val apiClient = ApiClient.getInstance()
    
    init {
        loadParkedVehicles()
    }
    
    fun loadParkedVehicles() {
        viewModelScope.launch {
            try {
                Log.d(TAG, "주차된 차량 목록 로딩 시작")
                when (val response = apiClient.getParkingStatus()) {
                    is ApiResponse.Success -> {
                        Log.d(TAG, "주차장 상태 조회 성공: ${response.data}")
                        response.data.parkedVehicles?.let {
                            Log.d(TAG, "주차된 차량 목록: $it")
                            _parkedVehicles.value = it
                            _errorMessage.value = null
                        }
                    }
                    is ApiResponse.Error -> {
                        Log.e(TAG, "주차장 상태 조회 실패: ${response.message}")
                        _errorMessage.value = response.message
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "주차된 차량 목록 로드 실패", e)
                _errorMessage.value = "주차 차량 목록 로드 실패: ${e.message}"
            }
        }
    }

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
                        _cameraStream.value = response
                        delay(100) // 100ms 딜레이로 프레임 레이트 조절
                    } catch (e: Exception) {
                        // 카메라 스트림 에러는 무시
                        delay(3000) // 에러 발생 시 3초 대기 후 재시도
                    }
                }
            } finally {
                _isStreaming.value = false
            }
        }
    }

    fun stopCameraStream() {
        cameraStreamJob?.cancel()
        cameraStreamJob = null
        _isStreaming.value = false
        _errorMessage.value = null
    }

    fun selectVehicle(vehicle: ParkedVehicle) {
        _selectedVehicle.value = vehicle
        _errorMessage.value = null
    }

    fun processImage(imageBase64: String) {
        viewModelScope.launch {
            try {
                when (val result = apiClient.getLatestOcr()) {
                    is ApiResponse.Success -> {
                        val ocr = result.data
                        val matchingVehicles = _parkedVehicles.value?.filter { 
                            it.license_plate == ocr.car_plate 
                        } ?: emptyList()
                        
                        if (matchingVehicles.isNotEmpty()) {
                            _selectedVehicle.value = matchingVehicles.first()
                            _errorMessage.value = null
                        } else {
                            _errorMessage.value = "일치하는 차량을 찾을 수 없습니다"
                        }
                    }
                    is ApiResponse.Error -> {
                        _errorMessage.value = result.message
                    }
                }
            } catch (e: Exception) {
                _errorMessage.value = "이미지 처리 실패: ${e.message}"
            }
        }
    }

    fun searchVehicles(query: String) {
        if (query.length != 4) {
            _matchingVehicles.value = emptyList()
            return
        }

        Log.d(TAG, "차량 검색 시작 - 검색어: $query")
        val currentList = _parkedVehicles.value ?: return
        Log.d(TAG, "현재 주차된 차량 목록: $currentList")
        
        val filteredList = currentList.filter { vehicle ->
            vehicle.license_plate.endsWith(query)
        }
        Log.d(TAG, "검색 결과: $filteredList")
        
        _matchingVehicles.value = filteredList
        when {
            filteredList.isEmpty() -> {
                _errorMessage.value = "일치하는 차량이 없습니다."
            }
            filteredList.size == 1 -> {
                showSingleVehicleConfirmation(filteredList.first())
            }
            else -> {
                _errorMessage.value = null
            }
        }
    }

    private fun showSingleVehicleConfirmation(vehicle: ParkedVehicle) {
        _selectedVehicle.value = vehicle
    }

    fun exitVehicle(vehicle: ParkedVehicle) {
        viewModelScope.launch {
            try {
                when (val response = apiClient.exitVehicle(ExitRequest(
                    license_plate = vehicle.license_plate,
                    car_type = vehicle.car_type
                ))) {
                    is ApiResponse.Success -> {
                        _exitResult.value = response.data
                        _selectedVehicle.value = null
                        _matchingVehicles.value = emptyList()
                        loadParkedVehicles() // 목록 갱신
                        _errorMessage.value = null
                    }
                    is ApiResponse.Error -> {
                        _errorMessage.value = response.message
                    }
                }
            } catch (e: Exception) {
                _errorMessage.value = "출차 처리 실패: ${e.message}"
            }
        }
    }

    override fun onCleared() {
        super.onCleared()
        stopCameraStream()
    }
} 