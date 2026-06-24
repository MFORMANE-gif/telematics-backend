/**
 * ============================================================
 *  TELEMATICS BACKEND — Simple HTTP Receiver + Live Dashboard
 * ============================================================
 *  Yeh server ESP32 firmware se JSON data accept karta hai
 *  POST request ke through, aur ek live dashboard page deta hai
 *  jahan latest vehicle data + GPS location dikhta hai.
 *
 *  ENDPOINTS:
 *  POST /data         -> ESP32 yahan JSON bhejega
 *  GET  /api/latest    -> Dashboard yahan se latest data fetch karega
 *  GET  /              -> Live dashboard HTML page
 * ============================================================
 */

const express = require('express');
const path = require('path');

const app = express();
const PORT = process.env.PORT || 3000;

// JSON body parser (ESP32 se aane wale POST data ke liye)
app.use(express.json());

// Static dashboard files serve karo
app.use(express.static(path.join(__dirname, 'public')));

// ============================================================
// IN-MEMORY DATA STORE
// ============================================================
// Simple setup ke liye database nahi, sirf memory mein latest
// data aur thodi history rakhते hain. Server restart hone par
// data clear ho jayega — yeh sirf testing/demo ke liye hai.
// ============================================================
let latestData = {
    driver_uid: "NO_CARD",
    lat: 0,
    lon: 0,
    gps_fix: false,
    network_online: false,
    speed_kmh: 0,
    rpm: 0,
    coolant_c: 0,
    oil_kpa: 0,
    odometer_km: 0,
    battery_v: 0,
    lng_level_pct: 0,
    lng_volume_l: 0,
    uptime_sec: 0,
    received_at: null
};

const MAX_HISTORY = 200;
let history = [];

// ============================================================
// POST /data — ESP32 yahan data bhejega
// ============================================================
app.post('/data', (req, res) => {
    const payload = req.body;

    if (!payload || typeof payload !== 'object') {
        return res.status(400).json({ ok: false, error: 'Invalid JSON body' });
    }

    latestData = {
        ...latestData,
        ...payload,
        received_at: new Date().toISOString()
    };

    history.push(latestData);
    if (history.length > MAX_HISTORY) {
        history.shift();
    }

    console.log(`[DATA] ${latestData.received_at} | Driver: ${latestData.driver_uid} | Speed: ${latestData.speed_kmh} km/h | Lat: ${latestData.lat} Lon: ${latestData.lon}`);

    res.status(200).json({ ok: true, message: 'Data received' });
});

// ============================================================
// GET /api/latest — Dashboard fetch karega yahan se
// ============================================================
app.get('/api/latest', (req, res) => {
    res.json(latestData);
});

// ============================================================
// GET /api/history — Pichla data (optional, route plotting ke liye)
// ============================================================
app.get('/api/history', (req, res) => {
    res.json(history);
});

// ============================================================
// Health check (Render/uptime monitors ke liye)
// ============================================================
app.get('/health', (req, res) => {
    res.json({ status: 'ok', uptime_sec: process.uptime() });
});

app.listen(PORT, () => {
    console.log(`================================================`);
    console.log(`  Telematics Backend chal raha hai`);
    console.log(`  Port: ${PORT}`);
    console.log(`  POST data yahan bhejo: /data`);
    console.log(`  Dashboard dekho: /`);
    console.log(`================================================`);
});
