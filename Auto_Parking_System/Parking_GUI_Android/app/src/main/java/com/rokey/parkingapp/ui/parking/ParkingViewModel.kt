package com.rokey.parkingapp.ui.parking

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.rokey.parkingapp.R
import com.rokey.parkingapp.network.*
import kotlinx.coroutines.launch
import android.util.Log

data class CarTypeInfo(
    val displayText: String,
    val iconResId: Int
)

class ParkingViewModel : ViewModel() {
    private val _ocrResult = MutableLiveData<OcrResult>()
    val ocrResult: LiveData<OcrResult> = _ocrResult

    private val _parkingLocation = MutableLiveData<String>()
    val parkingLocation: LiveData<String> = _parkingLocation

    private val _currentCarType = MutableLiveData<String>("normal")
    val currentCarType: LiveData<String> = _currentCarType

    private val _cameraStreamError = MutableLiveData<Boolean>(false)
    val cameraStreamError: LiveData<Boolean> = _cameraStreamError

    private val apiClient = ApiClient.getInstance()

    fun getCarTypeInfo(carType: String): CarTypeInfo {
        val info = when (carType.lowercase()) {
            "normal" -> CarTypeInfo("일반 차량", R.drawable.ic_car_normal)
            "ev" -> CarTypeInfo("전기 차량", R.drawable.ic_car_ev)
            "disabled" -> CarTypeInfo("장애인 차량", R.drawable.ic_car_disabled)
            else -> CarTypeInfo("일반 차량", R.drawable.ic_car_normal)
        }
        Log.d("ParkingViewModel", "차량 타입 정보: $carType -> ${info.displayText}")
        return info
    }

    fun updateOcrResult(result: OcrResult) {
        _ocrResult.postValue(result)
        _currentCarType.postValue(result.type.lowercase())
        Log.d("ParkingViewModel", "OCR 결과 업데이트: ${result.car_plate}, 타입: ${result.type}")
    }

    fun updateCarType(type: String) {
        _currentCarType.postValue(type.lowercase())
        Log.d("ParkingViewModel", "차량 타입 수동 업데이트: $type")
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
                val response = apiClient.parkVehicle(request)
                if (response is ApiResponse.Success) {
                    _parkingLocation.value = response.data.location
                }
            } catch (e: Exception) {
                Log.e("ParkingViewModel", "주차 처리 중 오류 발생", e)
            }
        }
    }
} 