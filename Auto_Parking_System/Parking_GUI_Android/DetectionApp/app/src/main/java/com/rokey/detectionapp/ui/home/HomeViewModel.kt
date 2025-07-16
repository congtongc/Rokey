package com.rokey.detectionapp.ui.home

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.rokey.detectionapp.network.ApiClient
import com.rokey.detectionapp.network.OcrResult
import com.rokey.detectionapp.network.ParkedVehicle
import com.rokey.detectionapp.network.Statistics
import kotlinx.coroutines.launch

class HomeViewModel : ViewModel() {
    
    private val _parkingStatus = MutableLiveData<ParkingStatus>()
    val parkingStatus: LiveData<ParkingStatus> = _parkingStatus
    
    fun loadParkingStatus() {
        viewModelScope.launch {
            try {
                val response = ApiClient.apiService.getStatus()
                if (response.isSuccessful) {
                    response.body()?.let {
                        _parkingStatus.value = ParkingStatus(
                            statistics = it.statistics,
                            parkedVehicles = it.parkedVehicles,
                            latestOcr = it.latestOcr
                        )
                    }
                }
            } catch (e: Exception) {
                // 에러 처리
                e.printStackTrace()
            }
        }
    }
}

// 데이터 클래스
data class ParkingStatus(
    val statistics: Statistics,
    val parkedVehicles: List<ParkedVehicle>,
    val latestOcr: OcrResult
) 