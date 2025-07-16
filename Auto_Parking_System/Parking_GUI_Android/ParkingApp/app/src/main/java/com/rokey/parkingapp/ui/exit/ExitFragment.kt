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
import android.text.Editable
import android.text.TextWatcher
import com.rokey.parkingapp.ui.common.VehicleListAdapter
import androidx.appcompat.app.AlertDialog
import com.rokey.parkingapp.R
import com.google.android.material.snackbar.Snackbar

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
        
        viewModel.matchingVehicles.observe(viewLifecycleOwner) { vehicles ->
            when {
                vehicles.isEmpty() -> {
                    binding.cardSelectedVehicle.visibility = View.GONE
                    binding.btnExit.isEnabled = false
                }
                vehicles.size == 1 -> {
                    showExitConfirmationDialog(vehicles.first())
                }
                vehicles.size > 1 -> {
                    showVehicleSelectionDialog(vehicles)
                }
            }
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
        binding.etSearch.addTextChangedListener(object : TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
            override fun afterTextChanged(s: Editable?) {
                s?.toString()?.let { 
                    if (it.length == 4) {
                        viewModel.searchVehicles(it)
                    }
                }
            }
        })

        binding.btnExit.setOnClickListener {
            viewModel.selectedVehicle.value?.let { vehicle ->
                showExitConfirmationDialog(vehicle)
            } ?: run {
                Toast.makeText(context, "출차할 차량을 선택해주세요", Toast.LENGTH_SHORT).show()
            }
        }
    }
    
    private fun updateSelectedVehicleUI(vehicle: ParkedVehicle?) {
        if (vehicle != null) {
            binding.cardSelectedVehicle.visibility = View.VISIBLE
            binding.tvLicensePlate.text = vehicle.license_plate
            
            // 차량 타입 설정
            val (displayText, iconRes) = when(vehicle.car_type.lowercase()) {
                "normal" -> "일반 차량" to R.drawable.ic_car_normal
                "ev" -> "전기 차량" to R.drawable.ic_car_ev
                "disabled" -> "장애인 차량" to R.drawable.ic_car_disabled
                else -> "일반 차량" to R.drawable.ic_car_normal
            }
            binding.tvCarType.text = displayText
            binding.ivCarTypeIcon.setImageResource(iconRes)
            binding.ivCarTypeIcon.visibility = View.VISIBLE
            
            binding.tvLocation.text = vehicle.location
            binding.tvEntryTime.text = vehicle.time
            binding.btnExit.isEnabled = true
        } else {
            binding.cardSelectedVehicle.visibility = View.GONE
            binding.btnExit.isEnabled = false
        }
    }

    private fun showVehicleSelectionDialog(vehicles: List<ParkedVehicle>) {
        val items = vehicles.map { vehicle ->
            val displayText = when(vehicle.car_type.lowercase()) {
                "normal" -> "일반 차량"
                "ev" -> "전기 차량"
                "disabled" -> "장애인 차량"
                else -> "일반 차량"
            }
            "${vehicle.license_plate} ($displayText)"
        }.toTypedArray()
        
        AlertDialog.Builder(requireContext())
            .setTitle("출차할 차량을 선택해주세요")
            .setItems(items) { dialog, which ->
                showExitConfirmationDialog(vehicles[which])
                dialog.dismiss()
            }
            .setNegativeButton("취소") { dialog, _ ->
                dialog.dismiss()
            }
            .show()
    }

    private fun showExitConfirmationDialog(vehicle: ParkedVehicle) {
        val carType = when(vehicle.car_type.lowercase()) {
            "normal" -> "일반 차량"
            "ev" -> "전기 차량"
            "disabled" -> "장애인 차량"
            else -> "일반 차량"
        }
        
        AlertDialog.Builder(requireContext())
            .setTitle("출차 확인")
            .setMessage("차량번호: ${vehicle.license_plate}\n차량종류: $carType\n위치: ${vehicle.location}\n\n이 차량을 출차하시겠습니까?")
            .setPositiveButton("확인") { dialog, _ ->
                viewModel.exitVehicle(vehicle)
                Snackbar.make(
                    binding.root,
                    "${vehicle.license_plate} 차량이 출차되었습니다.",
                    Snackbar.LENGTH_LONG
                ).show()
                dialog.dismiss()
            }
            .setNegativeButton("취소") { dialog, _ ->
                dialog.dismiss()
            }
            .show()
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