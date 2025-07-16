package com.rokey.detectionapp.network

import okhttp3.OkHttpClient
import okhttp3.logging.HttpLoggingInterceptor
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import retrofit2.http.GET
import java.util.concurrent.TimeUnit

object ApiConfig {
    const val BASE_URL = "http://192.168.0.100:8000" // 브릿지 서버 주소
    const val WS_BASE_URL = "ws://192.168.0.100:8000" // 웹소켓 주소
}

object ApiClient {
    private val okHttpClient = OkHttpClient.Builder()
        .addInterceptor(HttpLoggingInterceptor().apply {
            level = HttpLoggingInterceptor.Level.BODY
        })
        .connectTimeout(30, TimeUnit.SECONDS)
        .readTimeout(30, TimeUnit.SECONDS)
        .writeTimeout(30, TimeUnit.SECONDS)
        .build()
    
    private val retrofit = Retrofit.Builder()
        .baseUrl(ApiConfig.BASE_URL)
        .client(okHttpClient)
        .addConverterFactory(GsonConverterFactory.create())
        .build()
    
    val apiService: ApiService = retrofit.create(ApiService::class.java)
}

interface ApiService {
    @GET("/status")
    suspend fun getStatus(): Response<StatusResponse>
    
    @GET("/latest_ocr")
    suspend fun getLatestOcr(): Response<OcrResult>
}

// 데이터 클래스
data class StatusResponse(
    val status: String,
    val timestamp: String,
    val statistics: Statistics,
    val parkedVehicles: List<ParkedVehicle>,
    val latestOcr: OcrResult
)

data class Statistics(
    val total: SpaceCount,
    val occupied: SpaceCount,
    val available: SpaceCount
)

data class SpaceCount(
    val normal: Int,
    val ev: Int,
    val disabled: Int
)

data class ParkedVehicle(
    val license_plate: String,
    val car_type: String,
    val location: String,
    val time: String
)

data class OcrResult(
    val car_plate: String,
    val type: String
) 