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
import com.rokey.parkingapp.network.ApiResponse

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
    private val prefs = context.getSharedPreferences("server_prefs", Context.MODE_PRIVATE)
    private var retrofit: Retrofit
    private var baseUrl: String = ""
    private var apiService: ApiService

    init {
        // SharedPreferences에서 서버 주소 가져오기
        baseUrl = prefs.getString("base_url", null) ?: "http://localhost:8080/"
        
        val client = OkHttpClient.Builder()
            .connectTimeout(ApiConfig.CONNECT_TIMEOUT, TimeUnit.SECONDS)
            .readTimeout(ApiConfig.READ_TIMEOUT, TimeUnit.SECONDS)
            .writeTimeout(ApiConfig.WRITE_TIMEOUT, TimeUnit.SECONDS)
            .addInterceptor(RetryInterceptor())
            .build()

        retrofit = Retrofit.Builder()
            .baseUrl(baseUrl)
            .client(client)
            .addConverterFactory(GsonConverterFactory.create())
            .build()
            
        apiService = retrofit.create(ApiService::class.java)
    }

    fun updateBaseUrl(newUrl: String) {
        if (baseUrl == newUrl) return
        
        baseUrl = newUrl
        retrofit = retrofit.newBuilder()
            .baseUrl(newUrl)
            .build()
        
        apiService = retrofit.create(ApiService::class.java)
            
        // 새 URL을 SharedPreferences에 저장
        prefs.edit().putString("base_url", newUrl).apply()
    }

    companion object {
        @Volatile
        private var instance: ApiClient? = null

        fun initialize(context: Context) {
            if (instance == null) {
                synchronized(this) {
                    if (instance == null) {
                        instance = ApiClient(context.applicationContext)
                    }
                }
            }
        }

        fun getInstance(): ApiClient {
            return instance ?: throw IllegalStateException("ApiClient must be initialized")
        }

        suspend fun updateServerUrl(context: Context, newUrl: String): Boolean {
            return try {
                getInstance().updateBaseUrl(newUrl)
                true
            } catch (e: Exception) {
                Log.e("ApiClient", "서버 URL 업데이트 실패", e)
                false
            }
        }
    }

    suspend fun getParkingStatus(): ApiResponse<ParkingStatusResponse> {
        return try {
            val response = apiService.getParkingStatus()
            if (response.isSuccessful) {
                ApiResponse.Success(response.body()!!)
            } else {
                ApiResponse.Error("서버 오류: ${response.code()}")
            }
        } catch (e: Exception) {
            ApiResponse.Error(e.message ?: "알 수 없는 오류")
        }
    }

    suspend fun parkVehicle(request: ParkingRequest): ApiResponse<ParkingResponse> {
        return try {
            val response = apiService.parkVehicle(request)
            if (response.isSuccessful) {
                ApiResponse.Success(response.body()!!)
            } else {
                ApiResponse.Error("서버 오류: ${response.code()}")
            }
        } catch (e: Exception) {
            ApiResponse.Error(e.message ?: "알 수 없는 오류")
        }
    }

    suspend fun exitVehicle(request: ExitRequest): ApiResponse<ExitResponse> {
        return try {
            val response = apiService.exitVehicle(request)
            if (response.isSuccessful) {
                ApiResponse.Success(response.body()!!)
            } else {
                ApiResponse.Error("서버 오류: ${response.code()}")
            }
        } catch (e: Exception) {
            ApiResponse.Error(e.message ?: "알 수 없는 오류")
        }
    }

    suspend fun getLatestOcr(): ApiResponse<OcrResult> {
        return try {
            val response = apiService.getLatestOcr()
            if (response.isSuccessful) {
                ApiResponse.Success(response.body()!!)
            } else {
                ApiResponse.Error("서버 오류: ${response.code()}")
            }
        } catch (e: Exception) {
            ApiResponse.Error(e.message ?: "알 수 없는 오류")
        }
    }

    suspend fun getStream(): ApiResponse<String> {
        return try {
            val response = apiService.getCameraStream()
            if (response.isSuccessful) {
                response.body()?.data?.let { 
                    ApiResponse.Success("data:image/jpeg;base64," + android.util.Base64.encodeToString(it, android.util.Base64.DEFAULT))
                } ?: ApiResponse.Error("카메라 프레임이 없습니다")
            } else {
                ApiResponse.Error("서버 오류: ${response.code()}")
            }
        } catch (e: Exception) {
            ApiResponse.Error(e.message ?: "알 수 없는 오류")
        }
    }

    suspend fun getCameraFrame(): String {
        return try {
            val response = apiService.getCameraStream()
            if (response.isSuccessful) {
                response.body()?.data?.let { 
                    "data:image/jpeg;base64," + android.util.Base64.encodeToString(it, android.util.Base64.DEFAULT)
                } ?: throw IOException("카메라 프레임이 없습니다")
            } else {
                throw IOException("서버 오류: ${response.code()}")
            }
        } catch (e: Exception) {
            throw IOException("카메라 스트림 오류: ${e.message}")
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