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
import androidx.appcompat.app.AlertDialog
import com.google.android.material.textfield.TextInputEditText
import android.widget.RadioGroup
import com.rokey.parkingapp.R

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
        viewModel.ocrResult.observe(viewLifecycleOwner) { result ->
            result?.let { 
                binding.tvLicensePlate.text = "번호판: ${it.car_plate}"
                
                // 차량 타입 설정
                val displayText = when(it.type.lowercase()) {
                    "normal" -> "일반 차량"
                    "ev" -> "전기 차량"
                    "disabled" -> "장애인 차량"
                    else -> "일반 차량"
                }
                
                val iconRes = when(it.type.lowercase()) {
                    "normal" -> R.drawable.ic_car_normal
                    "ev" -> R.drawable.ic_car_ev
                    "disabled" -> R.drawable.ic_car_disabled
                    else -> R.drawable.ic_car_normal
                }
                
                binding.tvCarType.text = "차량 타입: $displayText"
                binding.ivCarTypeIcon.setImageResource(iconRes)
                binding.ivCarTypeIcon.visibility = View.VISIBLE
            }
        }

        viewModel.parkingLocation.observe(viewLifecycleOwner) { location ->
            binding.tvLocation.text = "할당된 주차 위치: ${location ?: "-"}"
            binding.btnPark.isEnabled = location != null
        }
        
        viewModel.isStreaming.observe(viewLifecycleOwner) { isStreaming ->
            if (!isStreaming) {
                binding.root.postDelayed({
                    viewModel.startCameraStream()
                }, 3000)
            }
        }

        viewModel.cameraStreamError.observe(viewLifecycleOwner) { error ->
            if (error) {
                binding.webViewCamera.visibility = View.GONE
            } else {
                binding.webViewCamera.visibility = View.VISIBLE
            }
        }

        viewModel.errorMessage.observe(viewLifecycleOwner) { message ->
            message?.let {
                binding.tvError.text = it
                binding.tvError.visibility = View.VISIBLE
            }
        }
    }
    
    private fun setupListeners() {
        binding.btnPark.setOnClickListener {
            viewModel.ocrResult.value?.let { result ->
                Snackbar.make(
                    binding.root,
                    "주차가 완료되었습니다. 위치: ${viewModel.parkingLocation.value}",
                    Snackbar.LENGTH_LONG
                ).show()
                binding.btnPark.isEnabled = false
            }
        }

        binding.btnManualInput.setOnClickListener {
            showManualInputDialog()
        }
    }

    private fun showManualInputDialog() {
        val dialogView = LayoutInflater.from(requireContext())
            .inflate(R.layout.dialog_manual_input, null)
        
        val dialog = AlertDialog.Builder(requireContext())
            .setTitle("차량 정보 수동 입력")
            .setView(dialogView)
            .setPositiveButton("확인") { dialog, _ ->
                val licensePlate = dialogView.findViewById<TextInputEditText>(R.id.etLicensePlate).text.toString()
                
                val carType = when (dialogView.findViewById<RadioGroup>(R.id.rgCarType).checkedRadioButtonId) {
                    R.id.rbEv -> "ev"
                    R.id.rbDisabled -> "disabled"
                    else -> "normal"
                }

                // 차량 번호 유효성 검사
                val pattern = """^(\d{2,3})([가-힣]{1})(\d{4})$""".toRegex()
                if (!licensePlate.matches(pattern)) {
                    Snackbar.make(binding.root, 
                        "올바른 차량번호 형식이 아닙니다.\n예시: 12가1234, 123나1234", 
                        Snackbar.LENGTH_LONG).show()
                    return@setPositiveButton
                }

                viewModel.processManualInput(licensePlate, carType)
                dialog.dismiss()
            }
            .setNegativeButton("취소") { dialog, _ ->
                dialog.dismiss()
            }
            .create()
        
        dialog.show()
    }
    
    private fun showError(message: String) {
        binding.tvError.apply {
            text = message
            visibility = View.VISIBLE
        }
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
