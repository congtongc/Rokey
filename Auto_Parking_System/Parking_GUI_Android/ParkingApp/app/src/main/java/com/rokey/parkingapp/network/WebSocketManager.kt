package com.rokey.parkingapp.network

import android.util.Log
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import java.util.concurrent.TimeUnit
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch

object WebSocketManager {
    private val TAG = "WebSocketManager"
    private var webSocket: WebSocket? = null
    private var wsUrl: String = "${ApiConfig.WS_BASE_URL}/ws"
    private val listeners = mutableListOf<WebSocketListener>()
    private val scope = CoroutineScope(Dispatchers.IO)
    private var isReconnecting = false
    private var reconnectAttempts = 0
    private val MAX_RECONNECT_ATTEMPTS = 3
    
    private val client = OkHttpClient.Builder()
        .connectTimeout(10, TimeUnit.SECONDS)
        .readTimeout(10, TimeUnit.SECONDS)
        .writeTimeout(10, TimeUnit.SECONDS)
        .build()
    
    fun init(baseUrl: String) {
        wsUrl = "${baseUrl}/ws"
        Log.d(TAG, "WebSocket URL 설정: $wsUrl")
        reconnectAttempts = 0
        connect()
    }
    
    fun connect() {
        if (isReconnecting) {
            Log.d(TAG, "이미 재연결 시도 중입니다.")
            return
        }
        
        if (webSocket != null) {
            Log.d(TAG, "이미 WebSocket 연결이 있습니다. 재연결을 위해 닫습니다.")
            webSocket?.close(1000, "재연결")
            webSocket = null
        }
        
        val request = Request.Builder()
            .url(wsUrl)
            .build()
        
        Log.d(TAG, "WebSocket 연결 시도: $wsUrl")
        
        webSocket = client.newWebSocket(request, object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                Log.d(TAG, "WebSocket 연결됨")
                isReconnecting = false
                reconnectAttempts = 0
                listeners.forEach { it.onOpen(webSocket, response) }
            }
            
            override fun onMessage(webSocket: WebSocket, text: String) {
                Log.d(TAG, "WebSocket 메시지 수신: $text")
                listeners.forEach { it.onMessage(webSocket, text) }
            }
            
            override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                Log.d(TAG, "WebSocket 닫는 중: $code, $reason")
                listeners.forEach { it.onClosing(webSocket, code, reason) }
            }
            
            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                Log.d(TAG, "WebSocket 닫힘: $code, $reason")
                listeners.forEach { it.onClosed(webSocket, code, reason) }
            }
            
            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                Log.e(TAG, "WebSocket 오류: ${t.message}", t)
                listeners.forEach { it.onFailure(webSocket, t, response) }
                
                when (response?.code) {
                    403 -> {
                        Log.e(TAG, "WebSocket 연결이 거부됨 (403 Forbidden)")
                        ConnectionState.updateError("서버에서 WebSocket 연결을 거부했습니다.")
                    }
                    404 -> {
                        Log.e(TAG, "WebSocket 엔드포인트를 찾을 수 없음 (404 Not Found)")
                        ConnectionState.updateError("WebSocket 서비스를 찾을 수 없습니다.")
                    }
                    else -> {
                        if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                            reconnect()
                        } else {
                            Log.e(TAG, "최대 재연결 시도 횟수 초과")
                            ConnectionState.updateError("서버 연결에 실패했습니다.")
                        }
                    }
                }
            }
        })
    }
    
    private fun reconnect() {
        if (isReconnecting) return
        
        isReconnecting = true
        reconnectAttempts++
        
        scope.launch {
            Log.d(TAG, "5초 후 WebSocket 재연결 시도 (시도 ${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})")
            delay(5000)
            connect()
        }
    }
    
    fun disconnect() {
        webSocket?.close(1000, "정상 종료")
        webSocket = null
        isReconnecting = false
        reconnectAttempts = 0
        Log.d(TAG, "WebSocket 연결 종료")
    }
    
    fun addListener(listener: WebSocketListener) {
        listeners.add(listener)
    }
    
    fun removeListener(listener: WebSocketListener) {
        listeners.remove(listener)
    }
    
    fun sendMessage(message: String): Boolean {
        return webSocket?.send(message) ?: false
    }
} 