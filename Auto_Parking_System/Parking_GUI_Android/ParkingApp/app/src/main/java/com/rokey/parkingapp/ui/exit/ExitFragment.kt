package com.rokey.parkingapp.ui.exit

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.fragment.app.Fragment
import androidx.fragment.app.viewModels
import androidx.recyclerview.widget.LinearLayoutManager
import com.rokey.parkingapp.databinding.FragmentExitBinding
import com.rokey.parkingapp.network.ParkedVehicle
import android.util.Base64
import android.graphics.BitmapFactory
import android.text.Editable
import android.text.TextWatcher
import com.rokey.parkingapp.ui.common.VehicleListAdapter

class ExitFragment : Fragment() {
    private var _binding: FragmentExitBinding? = null
    private val binding get() = _binding!!
    private val viewModel: ExitViewModel by viewModels()
    private lateinit var adapter: VehicleListAdapter
    
    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        _binding = FragmentExitBinding.inflate(inflater, container, false)
        return binding.root
    }
    
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        setupRecyclerView()
        setupObservers()
        setupListeners()
        
        // 초기 데이터 로드
        viewModel.loadParkedVehicles()
    }

    private fun setupRecyclerView() {
        adapter = VehicleListAdapter { vehicle ->
            viewModel.selectVehicle(vehicle)
        }
        binding.rvParkedVehicles.layoutManager = LinearLayoutManager(context)
        binding.rvParkedVehicles.adapter = adapter
    }

    private fun setupObservers() {
        viewModel.parkedVehicles.observe(viewLifecycleOwner) { vehicles ->
            adapter.submitList(vehicles)
        }
        
        viewModel.selectedVehicle.observe(viewLifecycleOwner) { vehicle ->
            updateSelectedVehicleUI(vehicle)
        }

        viewModel.errorMessage.observe(viewLifecycleOwner) { message ->
            message?.let {
                Toast.makeText(context, it, Toast.LENGTH_LONG).show()
            }
        }

        viewModel.cameraStream.observe(viewLifecycleOwner) { stream ->
            binding.webViewCamera.loadData(stream, "text/html", "UTF-8")
        }
    }

    private fun setupListeners() {
        binding.etSearch.setOnQueryTextListener(object : androidx.appcompat.widget.SearchView.OnQueryTextListener {
            override fun onQueryTextSubmit(query: String?): Boolean {
                query?.let { viewModel.searchVehicles(it) }
                return true
            }

            override fun onQueryTextChange(newText: String?): Boolean {
                newText?.let { viewModel.searchVehicles(it) }
                return true
            }
        })

        binding.btnExit.setOnClickListener {
            viewModel.selectedVehicle.value?.let { vehicle ->
                viewModel.exitVehicle(vehicle)
            } ?: run {
                Toast.makeText(context, "출차할 차량을 선택해주세요", Toast.LENGTH_SHORT).show()
        }
        }

        binding.btnRefresh.setOnClickListener {
                viewModel.loadParkedVehicles()
            }
        }
        
    private fun updateSelectedVehicleUI(vehicle: ParkedVehicle?) {
        if (vehicle != null) {
            binding.cardSelectedVehicle.visibility = View.VISIBLE
            binding.tvLicensePlate.text = vehicle.license_plate
            binding.tvCarType.text = vehicle.car_type
            binding.tvLocation.text = vehicle.location
            binding.tvEntryTime.text = vehicle.time
            binding.btnExit.isEnabled = true
        } else {
            binding.cardSelectedVehicle.visibility = View.GONE
            binding.btnExit.isEnabled = false
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
        _binding = null
    }
} 