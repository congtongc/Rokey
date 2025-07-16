package com.rokey.parkingapp.ui.common

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.rokey.parkingapp.databinding.ItemVehicleBinding
import com.rokey.parkingapp.network.ParkedVehicle
import com.rokey.parkingapp.R

class VehicleListAdapter(
    private val onItemClick: (ParkedVehicle) -> Unit
) : ListAdapter<ParkedVehicle, VehicleListAdapter.ViewHolder>(VehicleDiffCallback()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val binding = ItemVehicleBinding.inflate(
            LayoutInflater.from(parent.context),
            parent,
            false
        )
        return ViewHolder(binding)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        val vehicle = getItem(position)
        holder.bind(vehicle)
    }

    inner class ViewHolder(
        private val binding: ItemVehicleBinding
    ) : RecyclerView.ViewHolder(binding.root) {

        init {
            binding.root.setOnClickListener {
                val position = bindingAdapterPosition
                if (position != RecyclerView.NO_POSITION) {
                    onItemClick(getItem(position))
                }
            }
        }

        fun bind(vehicle: ParkedVehicle) {
            with(binding) {
                tvLicensePlate.text = vehicle.license_plate
                
                // 차량 타입 설정
                val (displayText, iconRes) = when(vehicle.car_type.lowercase()) {
                    "normal" -> "일반 차량" to R.drawable.ic_car_normal
                    "ev" -> "전기 차량" to R.drawable.ic_car_ev
                    "disabled" -> "장애인 차량" to R.drawable.ic_car_disabled
                    else -> "일반 차량" to R.drawable.ic_car_normal
                }
                
                tvCarType.text = displayText
                ivCarTypeIcon.setImageResource(iconRes)
                
                tvLocation.text = vehicle.location
                tvEntryTime.text = vehicle.time
            }
        }
    }
}

class VehicleDiffCallback : DiffUtil.ItemCallback<ParkedVehicle>() {
    override fun areItemsTheSame(oldItem: ParkedVehicle, newItem: ParkedVehicle): Boolean {
        return oldItem.license_plate == newItem.license_plate
    }

    override fun areContentsTheSame(oldItem: ParkedVehicle, newItem: ParkedVehicle): Boolean {
        return oldItem == newItem
    }
} 