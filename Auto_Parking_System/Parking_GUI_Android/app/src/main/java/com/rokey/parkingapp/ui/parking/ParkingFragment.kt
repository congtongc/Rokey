package com.rokey.parkingapp.ui.parking

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.fragment.app.viewModels
import com.rokey.parkingapp.databinding.FragmentParkingBinding
import com.rokey.parkingapp.R
import android.app.AlertDialog
import android.view.View.GONE
import android.view.View.VISIBLE

class ParkingFragment : Fragment() {
    private var _binding: FragmentParkingBinding? = null
    private val binding get() = _binding!!
    private val viewModel: ParkingViewModel by viewModels()

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
        setupObservers()
        setupUI()
        
        // 초기 차량 타입 설정
        binding.tvCarType.text = "일반 차량"
        binding.ivCarTypeIcon.setImageResource(R.drawable.ic_car_normal)
        binding.ivCarTypeIcon.visibility = VISIBLE
    }

    private fun setupObservers() {
        viewModel.ocrResult.observe(viewLifecycleOwner) { result ->
            result?.let { 
                binding.textLicensePlate.text = it.car_plate
                
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
                
                binding.tvCarType.text = displayText
                binding.ivCarTypeIcon.setImageResource(iconRes)
                binding.ivCarTypeIcon.visibility = VISIBLE
            }
        }

        viewModel.cameraStreamError.observe(viewLifecycleOwner) { error ->
            if (error) {
                binding.imageCamera.visibility = GONE
                binding.tvCameraError.visibility = VISIBLE
                binding.tvCameraError.text = "카메라 스트림을 찾을 수 없습니다. 카메라가 연결되어 있는지 확인해주세요."
            } else {
                binding.imageCamera.visibility = VISIBLE
                binding.tvCameraError.visibility = GONE
            }
        }
    }

    private fun setupUI() {
        binding.btnManualType.setOnClickListener {
            showCarTypeDialog()
        }

        binding.btnPark.setOnClickListener {
            val licensePlate = binding.textLicensePlate.text.toString()
            if (licensePlate.isNotEmpty()) {
                viewModel.parkVehicle(licensePlate, viewModel.getCurrentCarType())
            }
        }
    }

    private fun showCarTypeDialog() {
        val carTypes = arrayOf("일반 차량", "전기 차량", "장애인 차량")
        AlertDialog.Builder(requireContext())
            .setTitle("차량 타입 선택")
            .setItems(carTypes) { _, which ->
                val (type, displayText, iconRes) = when (which) {
                    0 -> Triple("normal", "일반 차량", R.drawable.ic_car_normal)
                    1 -> Triple("ev", "전기 차량", R.drawable.ic_car_ev)
                    2 -> Triple("disabled", "장애인 차량", R.drawable.ic_car_disabled)
                    else -> Triple("normal", "일반 차량", R.drawable.ic_car_normal)
                }
                
                binding.tvCarType.text = displayText
                binding.ivCarTypeIcon.setImageResource(iconRes)
                binding.ivCarTypeIcon.visibility = VISIBLE
                viewModel.updateCarType(type)
            }
            .show()
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
} 