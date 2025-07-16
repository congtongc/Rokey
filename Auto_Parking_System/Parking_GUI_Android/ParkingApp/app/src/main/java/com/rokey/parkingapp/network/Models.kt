package com.rokey.parkingapp.network

sealed class ApiResponse<out T> {
    data class Success<out T>(val data: T) : ApiResponse<T>()
    data class Error(val message: String) : ApiResponse<Nothing>()
}

data class ParkingStatusResponse(
    val status: String,
    val message: String,
    val server_time: String,
    val uptime: String,
    val statistics: ParkingStatistics? = null,
    val parkedVehicles: List<ParkedVehicle>? = null,
    val latest_ocr: OcrResult? = null
)

data class ParkingStatistics(
    val total_vehicles: Int,
    val available_spaces: Int,
    val occupied_spaces: Int
)

data class ParkedVehicle(
    val license_plate: String,
    val car_type: String,
    val location: String,
    val time: String
)

data class OcrResult(
    val car_plate: String,
    val type: String = "normal",
    val confidence: Double = 0.0,
    val timestamp: String = System.currentTimeMillis().toString()
)

data class ParkingRequest(
    val license_plate: String,
    val car_type: String = "normal"
)

data class ExitRequest(
    val license_plate: String,
    val car_type: String = "normal"
)

data class ParkingResponse(
    val status: String,
    val license_plate: String,
    val car_type: String,
    val location: String
)

data class ExitResponse(
    val status: String,
    val license_plate: String,
    val car_type: String,
    val location: String
)

data class VehicleInfoResponse(
    val license_plate: String,
    val car_type: String,
    val location: String,
    val time: String
)

data class ParkingPublishRequest(
    val location: String
)

data class ParkingPublishResponse(
    val status: String,
    val location: String
)

data class CameraStreamResponse(
    val data: ByteArray
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as CameraStreamResponse
        return data.contentEquals(other.data)
    }

    override fun hashCode(): Int {
        return data.contentHashCode()
    }
} 