package com.rokey.parkingapp.ui.parking

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.fragment.app.viewModels
import com.rokey.parkingapp.databinding.FragmentParkingBinding
import android.widget.Toast
import com.google.android.material.snackbar.Snackbar
import com.rokey.parkingapp.utils.PenguinLib

class ParkingFragment : Fragment() {
    private var _binding: FragmentParkingBinding? = null
    private val binding get() = _binding!!
    private val viewModel: ParkingViewModel by viewModels()
    private lateinit var penguinLib: PenguinLib
    
    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        _binding = FragmentParkingBinding.inflate(inflater, container, false)
        return binding.root
    }
    
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        setupPenguinLib()
        setupObservers()
        setupListeners()
        setupWebView()
        // 화면 진입 시 자동으로 카메라 스트림 시작
        viewModel.startCameraStream()
    }

    private fun setupPenguinLib() {
        try {
            penguinLib = PenguinLib()
            if (!penguinLib.initialize()) {
                showError("카메라 처리 라이브러리 초기화 실패")
            }
        } catch (e: Exception) {
            showError("카메라 처리 라이브러리 로딩 실패: ${e.message}")
        }
    }

    private fun setupWebView() {
        binding.webViewCamera.settings.apply {
            javaScriptEnabled = true
            domStorageEnabled = true
            allowFileAccess = true
        }
    }

    private fun setupObservers() {
        viewModel.cameraStream.observe(viewLifecycleOwner) { response ->
            response?.let {
                binding.webViewCamera.loadUrl(it.url)
                // URL이 업데이트될 때마다 자동으로 OCR 처리
                try {
                    val result = penguinLib.processImage(it.url)
                    viewModel.processOcrResult(result)
                } catch (e: Exception) {
                    showError("이미지 처리 실패: ${e.message}")
                }
            }
        }

        viewModel.ocrResult.observe(viewLifecycleOwner) { result ->
            result?.let {
                binding.layoutRecognitionResult.visibility = View.VISIBLE
                binding.tvLicensePlate.text = it.licensePlate
                binding.tvCarType.text = it.carType
                binding.btnPark.isEnabled = true
            }
        }

        viewModel.parkingLocation.observe(viewLifecycleOwner) { location ->
            location?.let {
                binding.tvLocation.text = it
                binding.btnPark.isEnabled = true
            }
        }

        viewModel.errorMessage.observe(viewLifecycleOwner) { message ->
            message?.let { showError(it) }
        }
        
        viewModel.isStreaming.observe(viewLifecycleOwner) { isStreaming ->
            if (!isStreaming) {
                // 스트리밍이 중단되면 3초 후 재시작 시도
                binding.root.postDelayed({
                    viewModel.startCameraStream()
                }, 3000)
            }
        }
    }
    
    private fun setupListeners() {
        binding.btnPark.setOnClickListener {
            viewModel.parkVehicle()
        }
    }
    
    private fun showError(message: String) {
        binding.tvError.apply {
            text = message
            visibility = View.VISIBLE
        }
        Snackbar.make(binding.root, message, Snackbar.LENGTH_LONG).show()
    }

    override fun onResume() {
        super.onResume()
        viewModel.startCameraStream()
                    }

    override fun onPause() {
        super.onPause()
        viewModel.stopCameraStream()
    }
    
    override fun onDestroyView() {
        super.onDestroyView()
        try {
            penguinLib.cleanup()
        } catch (e: Exception) {
            // 정리 중 오류는 무시
        }
        _binding = null
    }
} 
