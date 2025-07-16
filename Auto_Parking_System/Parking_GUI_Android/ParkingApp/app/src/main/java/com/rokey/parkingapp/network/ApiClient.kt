package com.rokey.parkingapp.network

import android.content.Context
import android.util.Log
import okhttp3.OkHttpClient
import okhttp3.logging.HttpLoggingInterceptor
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import retrofit2.http.*
import java.util.concurrent.TimeUnit
import okhttp3.Interceptor
import java.io.IOException
import java.net.SocketTimeoutException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.coroutines.delay
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withTimeout
import kotlinx.coroutines.withTimeoutOrNull
import java.net.InetAddress

object ApiConfig {
    private const val DEFAULT_PORT = 8000
    private const val LOCALHOST_PORT = 8000
    var BASE_URL: String = ""
    var WS_BASE_URL: String = ""
    const val CONNECT_TIMEOUT = 3L
    const val READ_TIMEOUT = 3L
    const val WRITE_TIMEOUT = 3L
    const val MAX_RETRIES = 1
    const val INITIAL_RETRY_DELAY = 500L
    const val MAX_RETRY_DELAY = 1000L
    const val RECONNECT_DELAY = 2000L
    const val SERVER_REACHABLE_TIMEOUT = 2000L
    const val MIN_RETRY_INTERVAL = 5000L
    
    private val serverAddresses = mutableListOf(
        "http://172.16.0.178:8000",
        "http://localhost:8000",
        "http://192.168.103.4:8000"
    )
    private var lastRetryTime = 0L
    private var currentServerIndex = 0
    
    fun updateUrls(httpUrl: String) {
        if (httpUrl.isEmpty()) {
            Log.w("ApiConfig", "서버 URL이 비어있어 기본 서버 목록에서 시도합니다.")
            tryNextServer()
            return
        }

        val url = normalizeUrl(httpUrl)
        BASE_URL = url
        WS_BASE_URL = url.replace("http://", "ws://")
        Log.d("ApiConfig", "서버 URL 업데이트: $BASE_URL")
    }

    fun normalizeUrl(url: String): String {
        var normalizedUrl = url
        if (!url.startsWith("http://") && !url.startsWith("https://")) {
            normalizedUrl = "http://$url"
        }
        if (!url.endsWith("/")) {
            normalizedUrl = "$normalizedUrl/"
        }
        return normalizedUrl
    }

    private fun tryNextServer() {
        if (currentServerIndex >= serverAddresses.size) {
            currentServerIndex = 0
        }
        val nextServer = serverAddresses[currentServerIndex]
        currentServerIndex++
        updateUrls(nextServer)
    }

    fun resetUrl() {
        BASE_URL = ""
        WS_BASE_URL = ""
        currentServerIndex = 0
        Log.d("ApiConfig", "서버 URL 초기화 (이전: $BASE_URL)")
    }

    fun canRetryNow(): Boolean {
        val currentTime = System.currentTimeMillis()
        return currentTime - lastRetryTime >= MIN_RETRY_INTERVAL
    }

    fun updateLastRetryTime() {
        lastRetryTime = System.currentTimeMillis()
    }
}

class RetryInterceptor : Interceptor {
    override fun intercept(chain: Interceptor.Chain): okhttp3.Response {
        var retryCount = 0
        var response: okhttp3.Response? = null
        var exception: IOException? = null
        
        while (retryCount < ApiConfig.MAX_RETRIES) {
            try {
                response = chain.proceed(chain.request())
                if (response.isSuccessful || response.code == 404) {
                    ConnectionState.updateConnectionState(true)
                    ConnectionState.updateError(null)
                    return response
                }
                response.close()
            } catch (e: IOException) {
                exception = e
                ConnectionState.updateConnectionState(false)
                ConnectionState.updateError(e.message)
                Log.w("RetryInterceptor", "재시도 ${retryCount + 1} 실패 (${e.message})")
            }
            retryCount++
            if (retryCount < ApiConfig.MAX_RETRIES) {
                Thread.sleep(500L * retryCount)
            }
        }
        
        throw exception ?: IOException("${retryCount}회 재시도 후 요청 실패")
    }
}

class ApiClient private constructor(context: Context) {
    private val TAG = "ApiClient"
    private val serverConfig = ServerConfig.getInstance(context)
    private var retrofit: Retrofit
    private var apiService: ApiService
    private val mutex = Mutex()
    
    init {
        val loggingInterceptor = HttpLoggingInterceptor().apply {
        level = HttpLoggingInterceptor.Level.BODY
    }
    
        val client = OkHttpClient.Builder()
        .addInterceptor(loggingInterceptor)
            .addInterceptor(RetryInterceptor())
            .connectTimeout(ApiConfig.CONNECT_TIMEOUT, TimeUnit.SECONDS)
            .readTimeout(ApiConfig.READ_TIMEOUT, TimeUnit.SECONDS)
            .writeTimeout(ApiConfig.WRITE_TIMEOUT, TimeUnit.SECONDS)
            .retryOnConnectionFailure(true)
        .build()

        // BASE_URL이 비어있을 경우 기본값 설정
        if (ApiConfig.BASE_URL.isEmpty()) {
            ApiConfig.BASE_URL = "http://localhost:8080/"
            Log.w(TAG, "서버 URL이 설정되지 않아 기본값으로 설정됨: ${ApiConfig.BASE_URL}")
        }

        // URL 형식 검증
        if (!ApiConfig.BASE_URL.startsWith("http://") && !ApiConfig.BASE_URL.startsWith("https://")) {
            ApiConfig.BASE_URL = "http://${ApiConfig.BASE_URL}"
            Log.w(TAG, "URL 스키마가 없어 'http://'를 추가함: ${ApiConfig.BASE_URL}")
        }

        // URL이 '/'로 끝나지 않으면 추가
        if (!ApiConfig.BASE_URL.endsWith("/")) {
            ApiConfig.BASE_URL += "/"
            Log.w(TAG, "URL 끝에 '/'를 추가함: ${ApiConfig.BASE_URL}")
        }

        retrofit = Retrofit.Builder()
            .baseUrl(ApiConfig.BASE_URL)
            .client(client)
            .addConverterFactory(GsonConverterFactory.create())
            .build()

        apiService = retrofit.create(ApiService::class.java)
    }

    companion object {
        private const val TAG = "ApiClient"
        @Volatile
        private var instance: ApiClient? = null
        private var applicationContext: Context? = null
        private var currentServerUrl: String? = null
        private var serverAutoDetectCallback: ((Boolean) -> Unit)? = null

        fun initialize(context: Context) {
            applicationContext = context.applicationContext
            if (instance == null) {
                synchronized(this) {
                    if (instance == null) {
                        instance = ApiClient(context.applicationContext)
                    }
                }
            }
        }

        fun setServerAutoDetectCallback(callback: (Boolean) -> Unit) {
            serverAutoDetectCallback = callback
        }

        fun getInstance(): ApiClient {
            return instance ?: throw IllegalStateException("ApiClient must be initialized first")
        }

        private suspend fun isServerReachable(address: String): Boolean = withContext(Dispatchers.IO) {
            try {
                val host = address.replace("http://", "").replace("https://", "").split(":")[0]
                var isReachable = false
                
                repeat(1) { attempt ->
                    try {
                        isReachable = withTimeoutOrNull(ApiConfig.SERVER_REACHABLE_TIMEOUT) {
                            val reachable = InetAddress.getByName(host).isReachable(1000)
                            Log.d(TAG, "서버 도달 가능 여부 체크 중: $host -> $reachable")
                            reachable
                        } ?: run {
                            Log.w(TAG, "서버 도달 가능 여부 체크 타임아웃")
                            false
                        }
                        
                        if (isReachable) {
                            Log.d(TAG, "서버 도달 가능 여부 체크 성공: $host")
                            return@repeat
                        }
                    } catch (e: Exception) {
                        Log.w(TAG, "서버 도달 가능 여부 체크 실패: ${e.message}")
                    }
                }
                
                if (!isReachable) {
                    Log.e(TAG, "서버 도달 불가: $host")
                }
                
                isReachable
                
            } catch (e: Exception) {
                Log.w(TAG, "서버 도달 가능 여부 체크 실패: ${e.message}")
                false
            }
        }

        private suspend fun testServerConnection(address: String): Boolean = withContext(Dispatchers.IO) {
            val formattedAddress = ApiConfig.normalizeUrl(address)
            Log.d(TAG, "서버 연결 테스트 시작: $formattedAddress")
            
            if (!isServerReachable(formattedAddress)) {
                Log.e(TAG, "서버에 도달할 수 없음: $formattedAddress")
                return@withContext false
            }

            var retryCount = 0
            var lastException: Exception? = null
            var currentDelay = ApiConfig.INITIAL_RETRY_DELAY
            
            while (retryCount < ApiConfig.MAX_RETRIES) {
                try {
                    val testClient = OkHttpClient.Builder()
                        .connectTimeout(ApiConfig.CONNECT_TIMEOUT, TimeUnit.SECONDS)
                        .readTimeout(ApiConfig.READ_TIMEOUT, TimeUnit.SECONDS)
                        .writeTimeout(ApiConfig.WRITE_TIMEOUT, TimeUnit.SECONDS)
                        .retryOnConnectionFailure(false)
                        .build()
                    
                    val request = okhttp3.Request.Builder()
                        .url("${formattedAddress}status")
                        .build()
                    
                    try {
                        withTimeoutOrNull(ApiConfig.CONNECT_TIMEOUT * 1000) {
                            val response = testClient.newCall(request).execute()
                            if (response.isSuccessful) {
                                Log.d(TAG, "서버 연결 성공: $formattedAddress")
                                return@withTimeoutOrNull true
                            }
                            Log.w(TAG, "서버 응답 실패 (HTTP ${response.code})")
                            false
                        } ?: run {
                            Log.w(TAG, "서버 연결 타임아웃")
                            false
                        }
                    } catch (e: Exception) {
                        lastException = e
                        Log.w(TAG, "서버 연결 시도 ${retryCount + 1} 실패: ${e.message}")
                        false
                    }
                    
                } catch (e: Exception) {
                    lastException = e
                    Log.w(TAG, "서버 연결 시도 ${retryCount + 1} 실패: ${e.message}")
                    false
                }
                
                retryCount++
                if (retryCount < ApiConfig.MAX_RETRIES) {
                    currentDelay = (currentDelay * 1.5).toLong().coerceAtMost(ApiConfig.MAX_RETRY_DELAY)
                    Log.d(TAG, "재시도 대기 중... (${currentDelay}ms)")
                    delay(currentDelay)
                }
            }
            
            if (lastException != null) {
                Log.e(TAG, "최종 연결 실패: ${lastException.message}")
            }
            false
        }

        suspend fun updateServerUrl(context: Context, address: String) {
            if (!ApiConfig.canRetryNow()) {
                Log.d(TAG, "최소 재시도 간격이 지나지 않아 서버 연결을 시도하지 않습니다.")
                return
            }
            
            Log.d(TAG, "서버 URL 업데이트 시도: $address (현재: $currentServerUrl)")
            
            if (address == currentServerUrl) {
                Log.d(TAG, "이미 같은 서버 주소를 사용 중입니다: $address")
                if (!testServerConnection(address)) {
                    Log.w(TAG, "현재 서버 연결 실패, 재설정 필요")
                    currentServerUrl = null
                    ApiConfig.resetUrl()
                }
                return
            }

            try {
                val formattedAddress = ApiConfig.normalizeUrl(address)
                val isConnected = testServerConnection(formattedAddress)
                if (!isConnected) {
                    throw IOException("서버 연결 실패: 최대 재시도 횟수 초과")
                }
                
                currentServerUrl = formattedAddress
                ApiConfig.updateUrls(formattedAddress)
                
                ServerConfig.getInstance(context).apply {
                    saveServerAddress(formattedAddress)
                }
                
                synchronized(this) {
                    instance = ApiClient(context.applicationContext)
                }
                
                WebSocketManager.disconnect()
                WebSocketManager.init(ApiConfig.WS_BASE_URL)
                
                ConnectionState.updateConnectionState(true)
                ConnectionState.updateError(null)
                
                Log.d(TAG, "서버 URL 업데이트 완료: $formattedAddress")
                
            } catch (e: Exception) {
                Log.e(TAG, "서버 연결 실패: ${e.message}")
                ConnectionState.updateConnectionState(false)
                ConnectionState.updateError("서버 연결 실패: ${e.message}")
                currentServerUrl = null
                ApiConfig.resetUrl()
                
                withContext(Dispatchers.Main) {
                    serverAutoDetectCallback?.invoke(false)
                }
                
                throw e
            }
        }

        suspend fun reconnect() {
            val context = applicationContext ?: return
            val serverConfig = ServerConfig.getInstance(context)
            val currentUrl = serverConfig.getServerUrl()
            
            if (currentUrl.isNotEmpty()) {
                delay(ApiConfig.RECONNECT_DELAY)
                try {
                    updateServerUrl(context, currentUrl)
                } catch (e: Exception) {
                    Log.e("ApiClient", "재연결 실패: ${e.message}")
                    withContext(Dispatchers.Main) {
                        serverAutoDetectCallback?.invoke(false)
                    }
                }
            }
        }
    }

    private suspend fun <T> executeWithRetry(
        action: suspend () -> T,
        errorHandler: (Exception) -> T
    ): T = mutex.withLock {
        try {
            action().also {
                ConnectionState.updateConnectionState(true)
                ConnectionState.updateError(null)
            }
        } catch (e: Exception) {
            ConnectionState.updateConnectionState(false)
            ConnectionState.updateError(e.message)
            
            reconnect()
            errorHandler(e)
        }
    }

    suspend fun getParkingStatus(): ParkingResponse = executeWithRetry(
        action = {
            val response = apiService.getParkingStatus()
            if (response.isSuccessful) {
                response.body()?.let { statusResponse ->
                    ParkingResponse(
                        success = true,
                        message = "Success",
                        parkedVehicles = statusResponse.parkedVehicles
                    )
                } ?: ParkingResponse(success = false, message = "응답이 비어있습니다", parkedVehicles = emptyList())
            } else {
                ParkingResponse(success = false, message = "API 호출 실패 (${response.code()})", parkedVehicles = emptyList())
            }
        },
        errorHandler = { e ->
            val errorMessage = when (e) {
                is SocketTimeoutException -> "서버 응답 시간 초과"
                is IOException -> "네트워크 오류: ${e.message}"
                else -> "알 수 없는 오류: ${e.message}"
            }
            ParkingResponse(success = false, message = errorMessage, parkedVehicles = emptyList())
        }
    )

    suspend fun startCameraStream(): CameraStreamResponse = withContext(Dispatchers.IO) {
        val response = apiService.getCameraStream()
        if (response.isSuccessful) {
            response.body() ?: CameraStreamResponse("", System.currentTimeMillis().toString())
        } else {
            throw IOException("Failed to get camera stream")
        }
    }

    suspend fun processOcr(imageBase64: String): OcrResult = withContext(Dispatchers.IO) {
        val response = apiService.getLatestOcr()
        if (response.isSuccessful) {
            response.body() ?: throw IOException("Empty OCR response")
        } else {
            throw IOException("Failed to process OCR")
        }
    }

    suspend fun parkVehicle(request: ParkingRequest): ApiResponse<Unit> = withContext(Dispatchers.IO) {
        val response = apiService.parkVehicle(request)
        if (response.isSuccessful) {
            ApiResponse(success = true, message = "주차 완료", data = Unit)
        } else {
            ApiResponse(success = false, message = "주차 실패", data = null)
        }
    }

    suspend fun processExit(request: ExitRequest): ExitResponse = withContext(Dispatchers.IO) {
        val response = apiService.exitVehicle(request)
        if (response.isSuccessful) {
            response.body() ?: ExitResponse(success = false, message = "Empty response")
        } else {
            ExitResponse(success = false, message = "Failed to process exit")
        }
    }

    suspend fun exitVehicle(licensePlate: String): ExitResponse = withContext(Dispatchers.IO) {
        try {
            val request = ExitRequest(licensePlate = licensePlate)
            val response = apiService.exitVehicle(request)
            if (response.isSuccessful) {
                response.body() ?: ExitResponse(success = false, message = "응답이 비어있습니다")
            } else {
                ExitResponse(success = false, message = "출차 처리 실패 (${response.code()})")
            }
        } catch (e: Exception) {
            val errorMessage = when (e) {
                is SocketTimeoutException -> "서버 응답 시간 초과"
                is IOException -> "네트워크 오류: ${e.message}"
                else -> "알 수 없는 오류: ${e.message}"
            }
            ExitResponse(success = false, message = errorMessage)
        }
    }
}

interface ApiService {
    @GET("/status")
    suspend fun getParkingStatus(): Response<ParkingStatusResponse>

    @GET("/status")
    suspend fun getServerStatus(): Response<ParkingStatusResponse>

    @GET("/")
    suspend fun getRootStatus(): Response<ParkingStatusResponse>
    
    @POST("/park")
    suspend fun parkVehicle(@Body request: ParkingRequest): Response<ParkingResponse>
    
    @POST("/exit")
    suspend fun exitVehicle(@Body request: ExitRequest): Response<ExitResponse>
    
    @GET("/latest_ocr")
    suspend fun getLatestOcr(): Response<OcrResult>

    @GET("/vehicle/info")
    suspend fun getVehicleInfo(@Query("plate_number") plateNumber: String): Response<VehicleInfoResponse>

    @POST("/publish_location")
    suspend fun publishParkingLocation(
        @Body request: ParkingPublishRequest
    ): Response<ParkingPublishResponse>

    @GET("/camera_frame")
    suspend fun getCameraStream(): Response<CameraStreamResponse>
} 