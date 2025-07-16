package com.rokey.parkingapp.ui.home

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.fragment.app.Fragment
import androidx.fragment.app.viewModels
import androidx.recyclerview.widget.LinearLayoutManager
import com.google.android.material.bottomnavigation.BottomNavigationView
import com.rokey.parkingapp.R
import com.rokey.parkingapp.databinding.FragmentHomeBinding
import com.rokey.parkingapp.network.ParkedVehicle
import com.rokey.parkingapp.network.ParkingStatistics
import com.rokey.parkingapp.ui.common.VehicleListAdapter

class HomeFragment : Fragment() {
    
    private var _binding: FragmentHomeBinding? = null
    private val binding get() = _binding!!
    
    private val viewModel: HomeViewModel by viewModels()
    private lateinit var adapter: VehicleListAdapter
    
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
        setupRecyclerView()
        setupObservers()
        setupRefreshLayout()
        viewModel.loadParkingStatus()
    }
    
    private fun setupRecyclerView() {
        adapter = VehicleListAdapter { vehicle ->
            // 차량 선택 처리
        }
        binding.rvParkedVehicles.layoutManager = LinearLayoutManager(context)
        binding.rvParkedVehicles.adapter = adapter
    }
    
    private fun setupObservers() {
        viewModel.statistics.observe(viewLifecycleOwner) { stats ->
            updateStatistics(stats)
        }

        viewModel.parkedVehicles.observe(viewLifecycleOwner) { vehicles ->
            adapter.submitList(vehicles)
            updateStatistics(viewModel.getStatistics())
        }
        
        // 에러 메시지 관찰
        viewModel.errorMessage.observe(viewLifecycleOwner) { message ->
            message?.let {
                Toast.makeText(context, it, Toast.LENGTH_LONG).show()
                binding.errorView.visibility = View.VISIBLE
                binding.tvError.text = it
                binding.btnRetry.visibility = View.VISIBLE
            } ?: run {
                binding.errorView.visibility = View.GONE
                binding.btnRetry.visibility = View.GONE
            }
        }
        
        // 로딩 상태 관찰
        viewModel.isLoading.observe(viewLifecycleOwner) { isLoading ->
            binding.swipeRefreshLayout.isRefreshing = isLoading
            binding.progressBar.visibility = if (isLoading) View.VISIBLE else View.GONE
        }
    }
    
    private fun setupRefreshLayout() {
        binding.swipeRefreshLayout.setOnRefreshListener {
        viewModel.loadParkingStatus()
            binding.swipeRefreshLayout.isRefreshing = false
        }
        
        binding.btnRetry.setOnClickListener {
            binding.errorView.visibility = View.GONE
            binding.progressBar.visibility = View.VISIBLE
            viewModel.retryConnection()
        }
        
        binding.btnPark.setOnClickListener {
            // 주차하기 화면으로 이동
            requireActivity().findViewById<BottomNavigationView>(R.id.nav_view)
                .selectedItemId = R.id.navigation_parking
        }
    }
    
    private fun updateStatistics(stats: Map<String, Map<String, Int>>) {
        with(binding) {
            // 전체 주차면
            val total = stats["total"] ?: mapOf("normal" to 2, "ev" to 2, "disabled" to 2)
            tvTotalSpots.text = "전체 주차면"
            tvTotalNormal.text = "${total["normal"] ?: 2}면"
            tvTotalEv.text = "${total["ev"] ?: 2}면"
            tvTotalDisabled.text = "${total["disabled"] ?: 2}면"

            // 사용 중인 주차면
            val occupied = stats["occupied"] ?: mapOf("normal" to 0, "ev" to 0, "disabled" to 0)
            tvOccupiedSpots.text = "사용 중"
            tvOccupiedNormal.text = "${occupied["normal"] ?: 0}면"
            tvOccupiedEv.text = "${occupied["ev"] ?: 0}면"
            tvOccupiedDisabled.text = "${occupied["disabled"] ?: 0}면"

            // 사용 가능한 주차면
            val available = stats["available"] ?: mapOf("normal" to 2, "ev" to 2, "disabled" to 2)
            tvAvailableSpots.text = "사용 가능"
            tvAvailableNormal.text = "${available["normal"] ?: 2}면"
            tvAvailableEv.text = "${available["ev"] ?: 2}면"
            tvAvailableDisabled.text = "${available["disabled"] ?: 2}면"

            // 프로그레스 바 업데이트
            pbTotal.max = 6  // 전체 주차면 수 (2 * 3)
            pbAvailable.max = 6
            pbOccupied.max = 6

            pbTotal.progress = (total["normal"] ?: 2) + (total["ev"] ?: 2) + (total["disabled"] ?: 2)
            pbAvailable.progress = (available["normal"] ?: 2) + (available["ev"] ?: 2) + (available["disabled"] ?: 2)
            pbOccupied.progress = (occupied["normal"] ?: 0) + (occupied["ev"] ?: 0) + (occupied["disabled"] ?: 0)

            // 퍼센트 텍스트 업데이트
            val totalPercent = ((pbTotal.progress.toFloat() / pbTotal.max) * 100).toInt()
            val availablePercent = ((pbAvailable.progress.toFloat() / pbAvailable.max) * 100).toInt()
            val occupiedPercent = ((pbOccupied.progress.toFloat() / pbOccupied.max) * 100).toInt()

            tvTotalPercent.text = "${totalPercent}%"
            tvAvailablePercent.text = "${availablePercent}%"
            tvOccupiedPercent.text = "${occupiedPercent}%"
        }
    }
    
    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
} 