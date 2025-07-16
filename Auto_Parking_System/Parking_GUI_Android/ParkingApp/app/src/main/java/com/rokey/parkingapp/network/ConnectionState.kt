package com.rokey.parkingapp.network

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

object ConnectionState {
    private val _isConnected = MutableStateFlow(false)
    val isConnected: StateFlow<Boolean> = _isConnected.asStateFlow()

    private val _lastError = MutableStateFlow<String?>(null)
    val lastError: StateFlow<String?> = _lastError.asStateFlow()

    fun updateConnectionState(connected: Boolean) {
        _isConnected.value = connected
    }

    fun updateError(error: String?) {
        _lastError.value = error
    }

    fun reset() {
        _isConnected.value = false
        _lastError.value = null
    }
} 