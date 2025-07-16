package com.rokey.parkingapp.network

data class ParkingStatusResponse(
    val success: Boolean,
    val message: String,
    val parkedVehicles: List<ParkedVehicle>
)

data class ParkingResponse(
    val success: Boolean,
    val message: String,
    val parkedVehicles: List<ParkedVehicle>
)

data class ParkedVehicle(
    val licensePlate: String,
    val carType: String,
    val location: String,
    val entryTime: String
)

data class Statistics(
    val totalSpots: Int,
    val availableSpots: Int,
    val occupiedSpots: Int
)

data class OcrResult(
    val licensePlate: String,
    val carType: String,
    val confidence: Double,
    val timestamp: String
)

data class ParkingRequest(
    val licensePlate: String,
    val carType: String,
    val location: String
)

data class ExitRequest(
    val licensePlate: String
)

data class ExitResponse(
    val success: Boolean,
    val message: String
)

data class VehicleInfoResponse(
    val success: Boolean,
    val message: String,
    val vehicle: ParkedVehicle?
)

data class ParkingPublishRequest(
    val licensePlate: String,
    val location: String
)

data class ParkingPublishResponse(
    val success: Boolean,
    val message: String
)

data class CameraStreamResponse(
    val url: String,
    val timestamp: String
)

data class ApiResponse<T>(
    val success: Boolean,
    val message: String,
    val data: T?
) 