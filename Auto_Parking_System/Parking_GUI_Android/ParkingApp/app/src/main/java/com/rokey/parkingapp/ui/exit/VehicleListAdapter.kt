package com.rokey.parkingapp.ui.exit

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.rokey.parkingapp.databinding.ItemVehicleBinding
import com.rokey.parkingapp.network.ParkedVehicle

class VehicleListAdapter(
    private val onItemClick: (ParkedVehicle) -> Unit
) : ListAdapter<ParkedVehicle, VehicleListAdapter.ViewHolder>(DiffCallback()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val binding = ItemVehicleBinding.inflate(
            LayoutInflater.from(parent.context),
            parent,
            false
        )
        return ViewHolder(binding, onItemClick)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        holder.bind(getItem(position))
    }

    class ViewHolder(
        private val binding: ItemVehicleBinding,
        private val onItemClick: (ParkedVehicle) -> Unit
    ) : RecyclerView.ViewHolder(binding.root) {

        fun bind(vehicle: ParkedVehicle) {
            binding.tvLicensePlate.text = vehicle.license_plate
            binding.tvCarType.text = vehicle.car_type
            binding.tvLocation.text = vehicle.location
            binding.root.setOnClickListener { onItemClick(vehicle) }
        }
    }

    private class DiffCallback : DiffUtil.ItemCallback<ParkedVehicle>() {
        override fun areItemsTheSame(oldItem: ParkedVehicle, newItem: ParkedVehicle): Boolean {
            return oldItem.license_plate == newItem.license_plate
        }

        override fun areContentsTheSame(oldItem: ParkedVehicle, newItem: ParkedVehicle): Boolean {
            return oldItem == newItem
        }
    }
} 