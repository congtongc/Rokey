package com.rokey.detectionapp.ui.detection

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import com.bumptech.glide.Glide
import com.bumptech.glide.load.engine.DiskCacheStrategy
import com.rokey.detectionapp.databinding.FragmentDetectionBinding
import com.rokey.detectionapp.network.ApiConfig
import com.rokey.detectionapp.network.OcrResult
import okhttp3.*
import org.json.JSONObject
import java.util.concurrent.TimeUnit

class DetectionFragment : Fragment() {
    
    private var _binding: FragmentDetectionBinding? = null
    private val binding get() = _binding!!
    
    private lateinit var viewModel: DetectionViewModel
    private var webSocket: WebSocket? = null
    
    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        _binding = FragmentDetectionBinding.inflate(inflater, container, false)
        return binding.root
    }
    
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        
        viewModel = ViewModelProvider(this)[DetectionViewModel::class.java]
        
        // 카메라 스트림 설정
        setupCameraStream()
        
        // OCR 결과 관찰
        viewModel.ocrResult.observe(viewLifecycleOwner) { result ->
            binding.textLicensePlate.text = result.car_plate
            binding.textCarType.text = when (result.type) {
                "normal" -> "일반 차량"
                "ev" -> "전기 차량"
                "disabled" -> "장애인 차량"
                else -> "일반 차량"
            }
        }
        
        // 웹소켓 연결
        connectWebSocket()
    }
    
    private fun setupCameraStream() {
        // Glide를 사용하여 카메라 스트림 표시
        context?.let {
            Glide.with(it)
                .load("${ApiConfig.BASE_URL}/camera_frame")
                .diskCacheStrategy(DiskCacheStrategy.NONE)
                .skipMemoryCache(true)
                .into(binding.imageCamera)
        }
    }
    
    private fun connectWebSocket() {
        val client = OkHttpClient.Builder()
            .pingInterval(30, TimeUnit.SECONDS)
            .build()
            
        val request = Request.Builder()
            .url("${ApiConfig.WS_BASE_URL}/ws/camera")
            .build()
        
        val listener = object : WebSocketListener() {
            override fun onMessage(webSocket: WebSocket, text: String) {
                try {
                    val json = JSONObject(text)
                    val carPlate = json.optString("car_plate", "")
                    val type = json.optString("type", "normal")
                    
                    activity?.runOnUiThread {
                        viewModel.updateOcrResult(OcrResult(carPlate, type))
                    }
                } catch (e: Exception) {
                    e.printStackTrace()
                }
            }
            
            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                super.onFailure(webSocket, t, response)
                // 웹소켓 연결 실패 시 재연결 시도
                activity?.runOnUiThread {
                    binding.textConnectionStatus.text = "연결 실패: ${t.message}"
                    binding.textConnectionStatus.visibility = View.VISIBLE
                }
                
                // 5초 후 재연결 시도
                view?.postDelayed({
                    connectWebSocket()
                }, 5000)
            }
            
            override fun onOpen(webSocket: WebSocket, response: Response) {
                super.onOpen(webSocket, response)
                activity?.runOnUiThread {
                    binding.textConnectionStatus.visibility = View.GONE
                }
            }
        }
        
        webSocket = client.newWebSocket(request, listener)
    }
    
    override fun onDestroyView() {
        super.onDestroyView()
        webSocket?.close(1000, "Fragment destroyed")
        _binding = null
    }
} 