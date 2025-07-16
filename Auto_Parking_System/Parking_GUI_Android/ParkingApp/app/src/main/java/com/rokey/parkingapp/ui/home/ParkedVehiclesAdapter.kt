package com.rokey.parkingapp.ui.home

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.rokey.parkingapp.R
import com.rokey.parkingapp.network.ParkedVehicle

class ParkedVehiclesAdapter : ListAdapter<ParkedVehicle, ParkedVehiclesAdapter.ViewHolder>(DiffCallback()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val view = LayoutInflater.from(parent.context)
            .inflate(R.layout.item_parked_vehicle, parent, false)
        return ViewHolder(view)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        val item = getItem(position)
        holder.bind(item)
    }

    class ViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView) {
        private val textLicensePlate: TextView = itemView.findViewById(R.id.textLicensePlate)
        private val textCarType: TextView = itemView.findViewById(R.id.textCarType)
        private val textLocation: TextView = itemView.findViewById(R.id.textLocation)
        private val textTime: TextView = itemView.findViewById(R.id.textTime)

        fun bind(vehicle: ParkedVehicle) {
            textLicensePlate.text = vehicle.licensePlate
            textCarType.text = when (vehicle.carType) {
                "normal" -> "일반 차량"
                "ev" -> "전기 차량"
                "disabled" -> "장애인 차량"
                else -> "일반 차량"
            }
            textLocation.text = "위치: ${vehicle.location}"
            textTime.text = "입차 시간: ${vehicle.entryTime}"
        }
    }

    private class DiffCallback : DiffUtil.ItemCallback<ParkedVehicle>() {
        override fun areItemsTheSame(oldItem: ParkedVehicle, newItem: ParkedVehicle): Boolean {
            return oldItem.licensePlate == newItem.licensePlate
        }

        override fun areContentsTheSame(oldItem: ParkedVehicle, newItem: ParkedVehicle): Boolean {
            return oldItem == newItem
        }
    }
} 