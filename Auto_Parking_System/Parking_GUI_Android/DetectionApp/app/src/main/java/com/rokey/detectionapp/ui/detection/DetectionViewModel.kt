package com.rokey.detectionapp.ui.detection

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.rokey.detectionapp.network.ApiClient
import com.rokey.detectionapp.network.OcrResult
import kotlinx.coroutines.launch

class DetectionViewModel : ViewModel() {
    
    private val _ocrResult = MutableLiveData<OcrResult>()
    val ocrResult: LiveData<OcrResult> = _ocrResult
    
    init {
        // 초기값 설정
        _ocrResult.value = OcrResult("", "normal")
        
        // 최신 OCR 결과 로드
        loadLatestOcr()
    }
    
    fun updateOcrResult(result: OcrResult) {
        _ocrResult.value = result
    }
    
    private fun loadLatestOcr() {
        viewModelScope.launch {
            try {
                val response = ApiClient.apiService.getLatestOcr()
                if (response.isSuccessful) {
                    response.body()?.let {
                        _ocrResult.value = it
                    }
                }
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
    }
} 