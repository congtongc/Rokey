package com.rokey.detectionapp.ui.home

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import com.rokey.detectionapp.MainActivity
import com.rokey.detectionapp.R
import com.rokey.detectionapp.databinding.FragmentHomeBinding
import com.rokey.detectionapp.ui.detection.DetectionFragment

class HomeFragment : Fragment() {
    
    private var _binding: FragmentHomeBinding? = null
    private val binding get() = _binding!!
    
    private lateinit var viewModel: HomeViewModel
    
    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        _binding = FragmentHomeBinding.inflate(inflater, container, false)
        return binding.root
    }
    
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        
        viewModel = ViewModelProvider(this)[HomeViewModel::class.java]
        
        // 차량 감지 시작 버튼 클릭 리스너
        binding.btnStartDetection.setOnClickListener {
            parentFragmentManager.beginTransaction()
                .replace(R.id.fragment_container, DetectionFragment())
                .commit()
            
            // 바텀 네비게이션 업데이트
            (activity as MainActivity).findViewById<com.google.android.material.bottomnavigation.BottomNavigationView>(
                R.id.bottom_navigation
            ).selectedItemId = R.id.navigation_detection
        }
        
        // 주차장 상태 정보 관찰
        viewModel.parkingStatus.observe(viewLifecycleOwner) { status ->
            updateParkingStatus(status)
        }
        
        // 주차장 상태 정보 로드
        viewModel.loadParkingStatus()
    }
    
    private fun updateParkingStatus(status: ParkingStatus) {
        binding.textTotalNormal.text = status.statistics.total.normal.toString()
        binding.textTotalEv.text = status.statistics.total.ev.toString()
        binding.textTotalDisabled.text = status.statistics.total.disabled.toString()
        
        binding.textOccupiedNormal.text = status.statistics.occupied.normal.toString()
        binding.textOccupiedEv.text = status.statistics.occupied.ev.toString()
        binding.textOccupiedDisabled.text = status.statistics.occupied.disabled.toString()
        
        binding.textAvailableNormal.text = status.statistics.available.normal.toString()
        binding.textAvailableEv.text = status.statistics.available.ev.toString()
        binding.textAvailableDisabled.text = status.statistics.available.disabled.toString()
    }
    
    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
} 