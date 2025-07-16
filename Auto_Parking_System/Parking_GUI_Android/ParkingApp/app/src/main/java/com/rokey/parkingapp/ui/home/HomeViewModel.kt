package com.rokey.parkingapp.ui.home

import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.rokey.parkingapp.network.*
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.io.IOException
import java.net.SocketTimeoutException

class HomeViewModel : ViewModel() {
    
    private val TAG = "HomeViewModel"
    
    private val _statistics = MutableLiveData<Map<String, Map<String, Int>>>()
    val statistics: LiveData<Map<String, Map<String, Int>>> = _statistics
    
    private val _parkedVehicles = MutableLiveData<List<ParkedVehicle>>()
    val parkedVehicles: LiveData<List<ParkedVehicle>> = _parkedVehicles
    
    private val _latestOcr = MutableLiveData<OcrResult>()
    val latestOcr: LiveData<OcrResult> = _latestOcr
    
    private val _errorMessage = MutableLiveData<String?>()
    val errorMessage: LiveData<String?> = _errorMessage
    
    private val _isLoading = MutableLiveData<Boolean>()
    val isLoading: LiveData<Boolean> = _isLoading
    
    private var autoRefreshJob: Job? = null
    private val AUTO_REFRESH_INTERVAL = 5000L // 5초마다 자동 갱신
    
    private val apiClient = ApiClient.getInstance()
    
    init {
        // 초기 데이터 설정
        initDefaultParkingStatus()
        
        // 자동 갱신 시작
        startAutoRefresh()
    }
    
    private fun initDefaultParkingStatus() {
        _statistics.value = mapOf(
            "total" to mapOf(
                "normal" to 2,
                "ev" to 2,
                "disabled" to 2
            ),
            "occupied" to mapOf(
                "normal" to 0,
                "ev" to 0,
                "disabled" to 0
            ),
            "available" to mapOf(
                "normal" to 2,
                "ev" to 2,
                "disabled" to 2
            )
        )
        _parkedVehicles.value = emptyList()
        _latestOcr.value = OcrResult(
            car_plate = "",
            type = "normal"
        )
    }
    
    fun loadParkingStatus() {
        viewModelScope.launch {
            try {
                _isLoading.value = true
                _errorMessage.value = null
                
                when (val response = apiClient.getParkingStatus()) {
                    is ApiResponse.Success -> {
                        response.data.parkedVehicles?.let { vehicles ->
                            _parkedVehicles.value = vehicles
                            
                            // 주차된 차량 타입별로 카운트
                            val occupiedCount = mutableMapOf(
                                "normal" to 0,
                                "ev" to 0,
                                "disabled" to 0
                            )
                            
                            vehicles.forEach { vehicle ->
                                occupiedCount[vehicle.car_type] = (occupiedCount[vehicle.car_type] ?: 0) + 1
                            }
                            
                            // 통계 업데이트
                            _statistics.value = mapOf(
                                "total" to mapOf(
                                    "normal" to 2,
                                    "ev" to 2,
                                    "disabled" to 2
                                ),
                                "occupied" to occupiedCount,
                                "available" to mapOf(
                                    "normal" to (2 - (occupiedCount["normal"] ?: 0)),
                                    "ev" to (2 - (occupiedCount["ev"] ?: 0)),
                                    "disabled" to (2 - (occupiedCount["disabled"] ?: 0))
                                )
                            )
                        }
                        
                        response.data.latest_ocr?.let { ocr ->
                            _latestOcr.value = ocr
                        }
                    }
                    is ApiResponse.Error -> {
                        _errorMessage.value = response.message
                    }
                }
            } catch (e: IOException) {
                _errorMessage.value = "네트워크 오류: ${e.message}"
            } catch (e: Exception) {
                _errorMessage.value = "오류 발생: ${e.message}"
            } finally {
                _isLoading.value = false
            }
        }
    }
    
    private fun startAutoRefresh() {
        autoRefreshJob?.cancel()
        autoRefreshJob = viewModelScope.launch {
            while (isActive) {
                delay(AUTO_REFRESH_INTERVAL)
                loadParkingStatus()
            }
        }
    }
    
    fun retryConnection() {
        loadParkingStatus()
    }
    
    fun getStatistics(): Map<String, Map<String, Int>> {
        return _statistics.value ?: mapOf(
            "total" to mapOf("normal" to 2, "ev" to 2, "disabled" to 2),
            "occupied" to mapOf("normal" to 0, "ev" to 0, "disabled" to 0),
            "available" to mapOf("normal" to 2, "ev" to 2, "disabled" to 2)
        )
    }
    
    fun getParkedVehicles(): List<ParkedVehicle> {
        return _parkedVehicles.value ?: emptyList()
    }
    
    fun getLatestOcrResult(): OcrResult? {
        return _latestOcr.value
    }
    
    override fun onCleared() {
        super.onCleared()
        autoRefreshJob?.cancel()
    }
} 