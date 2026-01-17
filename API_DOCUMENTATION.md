# QYH Jushen Robot API 鎺ュ彛鏂囨。

> **鐗堟湰**: 2.0.0  
> **鏇存柊鏃ユ湡**: 2026-01-15  
> **鍩虹URL**: `http://<robot_ip>:8000/api/v1`
> 
> **鈿狅笍 閲嶈鏇存柊**: v2.0 鐗堟湰缁熶竴浜嗘墍鏈?API 璺敱鍓嶇紑涓?`/api/v1`锛屽苟閲囩敤缁熶竴鍝嶅簲鏍煎紡銆傛棫璺敱 `/api/*` 浠嶇劧鍙敤浣嗗凡鏍囪涓?deprecated銆?

---

## 鐩綍

1. [姒傝堪](#1-姒傝堪)
2. [缁熶竴鍝嶅簲鏍煎紡](#2-缁熶竴鍝嶅簲鏍煎紡)
3. [璁よ瘉鏈哄埗](#3-璁よ瘉鏈哄埗)
4. [REST API 鎺ュ彛](#4-rest-api-鎺ュ彛)
5. [WebSocket 鎺ュ彛](#5-websocket-鎺ュ彛)
6. [閿欒鐮佽鏄嶿(#6-閿欒鐮佽鏄?
7. [琛屼笟鏈€浣冲疄璺礭(#7-琛屼笟鏈€浣冲疄璺?
8. [SDK 闆嗘垚绀轰緥](#8-sdk-闆嗘垚绀轰緥)

---

## 1. 姒傝堪

### 1.1 鏋舵瀯璁捐

```
鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
鈹?                    澶栭儴绯荤粺 / 瀹㈡埛绔?                        鈹?
鈹? (Web鍓嶇銆佺Щ鍔ˋpp銆丮ES绯荤粺銆佺涓夋柟闆嗘垚)                       鈹?
鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
                              鈹?
                    鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈻尖攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
                    鈹?  HTTP/WebSocket   鈹?
                    鈹?  (瀵瑰缃戝叧灞?      鈹?
                    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
                              鈹?
鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈻尖攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
鈹?                   QYH Jushen Backend                        鈹?
鈹? 鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?鈹?
鈹? 鈹? REST API    鈹? WebSocket   鈹? 瀹夊叏/鎺у埗鏉冪鐞?          鈹?鈹?
鈹? 鈹? (鎺у埗/閰嶇疆)  鈹? (瀹炴椂鎺ㄩ€?   鈹? (璁よ瘉銆佹€ュ仠銆佺湅闂ㄧ嫍)      鈹?鈹?
鈹? 鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹攢鈹€鈹€鈹€鈹€鈹€鈹€鈹粹攢鈹€鈹€鈹€鈹€鈹€鈹攢鈹€鈹€鈹€鈹€鈹€鈹€鈹粹攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?鈹?
鈹?        鈹?             鈹?                    鈹?             鈹?
鈹?        鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹尖攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?             鈹?
鈹?                       鈹?                                   鈹?
鈹?             鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈻尖攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                         鈹?
鈹?             鈹?  ROS2 Bridge     鈹?                         鈹?
鈹?             鈹?  (鍐呴儴閫氫俊)       鈹?                         鈹?
鈹?             鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?                         鈹?
鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹尖攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
                         鈹?
           鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈻尖攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
           鈹?  ROS2 / 纭欢椹卞姩灞?       鈹?
           鈹?  (搴曠洏銆佹満姊拌噦銆佷紶鎰熷櫒)    鈹?
           鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
```

### 1.2 鎺ュ彛鍗忚

| 鍗忚 | 鐢ㄩ€?| 鐗圭偣 |
|------|------|------|
| **REST/HTTP** | 鎺у埗鍛戒护銆侀厤缃€佷换鍔＄鐞?| 璇锋眰-鍝嶅簲妯″紡锛岄€傚悎浣庨鎿嶄綔 |
| **WebSocket** | 瀹炴椂鐘舵€佹帹閫併€侀仴鎿嶄綔 | 鍙屽悜閫氫俊锛?0Hz 鐘舵€佹帹閫?|

### 1.3 API 鏂囨。

鏈悗绔泦鎴愪簡 **Swagger UI** 鑷姩鏂囨。锛?
- Swagger UI: `http://<robot_ip>:8000/docs`
- ReDoc: `http://<robot_ip>:8000/redoc`
- OpenAPI JSON: `http://<robot_ip>:8000/openapi.json`

### 1.4 API 鐗堟湰

| 鐗堟湰 | 璺敱鍓嶇紑 | 鐘舵€?|
|------|----------|------|
| v1 (鎺ㄨ崘) | `/api/v1/*` | 褰撳墠鐗堟湰 |
| legacy | `/api/*` | deprecated锛屽皢鍦?v3 绉婚櫎 |

---

## 2. 缁熶竴鍝嶅簲鏍煎紡

### 2.1 鎴愬姛鍝嶅簲

鎵€鏈?API 杩斿洖缁熶竴鐨?JSON 鏍煎紡锛?

```json
{
  "success": true,
  "code": 0,
  "message": "鎿嶄綔鎴愬姛",
  "data": { ... },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

| 瀛楁 | 绫诲瀷 | 璇存槑 |
|------|------|------|
| `success` | boolean | 鎿嶄綔鏄惁鎴愬姛 |
| `code` | int | 涓氬姟鐘舵€佺爜锛? 琛ㄧず鎴愬姛 |
| `message` | string | 鍝嶅簲娑堟伅 |
| `data` | object/null | 涓氬姟鏁版嵁 |
| `timestamp` | string | ISO8601 鏃堕棿鎴?|

### 2.2 閿欒鍝嶅簲

```json
{
  "success": false,
  "code": 1101,
  "message": "鐢ㄦ埛鍚嶆垨瀵嗙爜閿欒",
  "data": null,
  "timestamp": "2026-01-15T10:30:00.000Z",
  "error": {
    "code": "AUTH_INVALID_CREDENTIALS",
    "message": "鐢ㄦ埛鍚嶆垨瀵嗙爜閿欒",
    "field": null,
    "details": null
  }
}
```

### 2.3 鍒嗛〉鍝嶅簲

```json
{
  "success": true,
  "code": 0,
  "message": "",
  "data": {
    "items": [...],
    "total": 100
  },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

---

## 3. 璁よ瘉鏈哄埗

### 3.1 JWT Token 璁よ瘉

鎵€鏈?API锛堥櫎鐧诲綍澶栵級閮介渶瑕?JWT Token 璁よ瘉銆?

#### 鐧诲綍鑾峰彇 Token

```http
POST /api/v1/auth/login
Content-Type: application/json

{
  "username": "admin",
  "password": "admin123"
}
```

**鍝嶅簲:**
```json
{
  "success": true,
  "code": 0,
  "message": "鐧诲綍鎴愬姛",
  "data": {
    "access_token": "eyJhbGciOiJIUzI1NiIs...",
    "token_type": "bearer",
    "expires_in": 1800,
    "user": {
      "id": 1,
      "username": "admin",
      "role": "admin",
      "email": "admin@example.com"
    }
  },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

#### 浣跨敤 Token

```http
GET /api/v1/robot/status
Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
```

#### 鍒锋柊 Token

```http
POST /api/v1/auth/refresh
Authorization: Bearer <current_token>
```

**鍝嶅簲:**
```json
{
  "success": true,
  "code": 0,
  "message": "Token 宸插埛鏂?,
  "data": {
    "refreshed": true,
    "access_token": "eyJhbGciOiJIUzI1NiIs...",
    "token_type": "bearer",
    "expires_in": 1800
  },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

### 3.2 瑙掕壊鏉冮檺

| 瑙掕壊 | 鏉冮檺璇存槑 |
|------|----------|
| `admin` | 瀹屾暣鏉冮檺锛屽寘鎷郴缁熼厤缃€佺敤鎴风鐞?|
| `operator` | 鎺у埗鏈哄櫒浜恒€佹墽琛屼换鍔?|
| `viewer` | 鍙璁块棶锛屾煡鐪嬬姸鎬?|

---

## 4. REST API 鎺ュ彛

### 4.1 绯荤粺鐘舵€?

#### 鍋ュ悍妫€鏌?

```http
GET /health
```

**鍝嶅簲:**
```json
{
  "success": true,
  "code": 0,
  "message": "绯荤粺杩愯姝ｅ父",
  "data": {
    "status": "healthy",
    "ros2_connected": true,
    "database": "ok"
  },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

#### 鑾峰彇 API 鐗堟湰淇℃伅

```http
GET /api/v1
```

**鍝嶅簲:**
```json
{
  "success": true,
  "code": 0,
  "message": "QYH Jushen Robot API",
  "data": {
    "version": "2.0.0",
    "api_prefix": "/api/v1"
  },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

#### 鑾峰彇鏈哄櫒浜虹姸鎬?

```http
GET /api/v1/robot/status
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "success": true,
  "code": 0,
  "message": "",
  "data": {
    "timestamp": "2026-01-15T10:30:00Z",
    "joints": {
      "left_arm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "right_arm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    },
    "grippers": {
      "left": {"position": 0.05, "force": 10.0},
      "right": {"position": 0.08, "force": 12.0}
    },
    "base": {
      "x": 0.0, "y": 0.0, "theta": 0.0,
      "velocity": {"linear": 0.0, "angular": 0.0}
    },
    "system": {
      "cpu_temp": 65.0,
      "battery": 85.0,
      "mode": "idle"
    }
  },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

---

### 4.2 鎺у埗鏉冪鐞?

> **閲嶈**: 鎵ц杩愬姩鍛戒护鍓嶅繀椤诲厛鑾峰彇鎺у埗鏉?

#### 鑾峰彇鎺у埗鏉?

```http
POST /api/v1/control/acquire
Authorization: Bearer <token>
Content-Type: application/json

{
  "duration": 300
}
```

**鍝嶅簲:**
```json
{
  "success": true,
  "code": 0,
  "message": "鎺у埗鏉冭幏鍙栨垚鍔?,
  "data": {
    "holder": {
      "user_id": 1,
      "username": "admin",
      "acquired_at": "2026-01-15T10:30:00Z",
      "expires_at": "2026-01-15T10:35:00Z"
    }
  },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

#### 閲婃斁鎺у埗鏉?

```http
POST /api/v1/control/release
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "success": true,
  "code": 0,
  "message": "鎺у埗鏉冨凡閲婃斁",
  "data": null,
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

#### 缁害鎺у埗鏉?

```http
POST /api/v1/control/renew
Authorization: Bearer <token>
Content-Type: application/json

{
  "duration": 300
}
```

#### 鏌ヨ鎺у埗鏉冪姸鎬?

```http
GET /api/v1/control/status
```

**鍝嶅簲:**
```json
{
  "success": true,
  "code": 0,
  "message": "",
  "data": {
    "locked": true,
    "holder": {
      "user_id": 1,
      "username": "admin",
      "acquired_at": "2026-01-15T10:30:00Z",
      "expires_at": "2026-01-15T10:35:00Z"
    }
  },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

---

### 4.3 鏈烘鑷傛帶鍒?

#### 鑾峰彇鏈烘鑷傜姸鎬?

```http
GET /api/v1/arm/state
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "connected": true,
  "robot_ip": "192.168.1.10",
  "powered_on": true,
  "enabled": true,
  "in_estop": false,
  "in_error": false,
  "servo_mode_enabled": false,
  "error_message": "",
  "left_in_position": true,
  "right_in_position": true,
  "left_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "right_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "left_cartesian_pose": {"x": 0.3, "y": 0.0, "z": 0.5, "rx": 0, "ry": 0, "rz": 0},
  "right_cartesian_pose": {"x": 0.3, "y": 0.0, "z": 0.5, "rx": 0, "ry": 0, "rz": 0}
}
```

#### 鏈烘鑷傝繛鎺?鏂紑

```http
POST /api/v1/arm/connect
Authorization: Bearer <token>
```

```http
POST /api/v1/arm/disconnect
Authorization: Bearer <token>
```

#### 鏈烘鑷備笂鐢?涓嬬數

```http
POST /api/v1/arm/power_on
Authorization: Bearer <token>
```

```http
POST /api/v1/arm/power_off
Authorization: Bearer <token>
```

#### 鏈烘鑷備娇鑳?鍘讳娇鑳?

```http
POST /api/v1/arm/enable
Authorization: Bearer <token>
```

```http
POST /api/v1/arm/disable
Authorization: Bearer <token>
```

> **娉ㄦ剰**: 浣胯兘/鍘讳娇鑳芥帴鍙ｆ棤闇€鍙傛暟锛屼細鍚屾椂鎿嶄綔鍙岃噦

#### 鍏宠妭杩愬姩 (MoveJ)

```http
POST /api/v1/arm/movej
Authorization: Bearer <token>
Content-Type: application/json

{
  "robot_id": -1,
  "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "velocity": 0.5,
  "acceleration": 0.3,
  "is_block": true
}
```

#### 鐩寸嚎杩愬姩 (MoveL)

```http
POST /api/v1/arm/movel
Authorization: Bearer <token>
Content-Type: application/json

{
  "robot_id": 0,
  "x": 0.3,
  "y": 0.0,
  "z": 0.5,
  "rx": 0.0,
  "ry": 0.0,
  "rz": 0.0,
  "velocity": 100.0,
  "acceleration": 50.0,
  "is_block": true
}
```

#### 鐐瑰姩鎺у埗 (Jog)

```http
POST /api/v1/arm/jog
Authorization: Bearer <token>
Content-Type: application/json

{
  "robot_id": 0,
  "axis_num": 1,
  "move_mode": 1,
  "coord_type": 1,
  "velocity": 0.1,
  "position": 0.01
}
```

| move_mode | 璇存槑 |
|-----------|------|
| 0 | 缁濆 |
| 1 | 姝ヨ繘 (INCR) |
| 2 | 杩炵画 (CONTINUE) |

---

### 3.4 澶圭埅鎺у埗

#### 鑾峰彇澶圭埅鐘舵€?

```http
GET /api/v1/gripper/state
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "left": {
    "is_activated": true,
    "is_moving": false,
    "object_status": 2,
    "object_status_text": "澶栧す鎶撳埌",
    "current_position": 150,
    "current_force": 50
  },
  "right": {
    "is_activated": true,
    "is_moving": false,
    "object_status": 3,
    "object_status_text": "鍒拌揪浣嶇疆",
    "current_position": 0,
    "current_force": 0
  }
}
```

#### 鎺у埗澶圭埅

```http
POST /api/v1/gripper/move
Authorization: Bearer <token>
Content-Type: application/json

{
  "side": "left",
  "position": 255,
  "speed": 255,
  "force": 150
}
```

| 鍙傛暟 | 鑼冨洿 | 璇存槑 |
|------|------|------|
| position | 0-255 | 0=鍏ㄥ紑, 255=鍏ㄩ棴 |
| speed | 0-255 | 杩愬姩閫熷害 |
| force | 0-255 | 澶规寔鍔?|

---

### 3.5 搴曠洏鎺у埗

#### 鑾峰彇搴曠洏鐘舵€?

```http
GET /api/v1/chassis/status
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "connected": true,
  "system_status": 2,
  "system_status_text": "绯荤粺绌洪棽",
  "location_status": 3,
  "location_status_text": "瀹氫綅鎴愬姛",
  "current_map": "warehouse_floor1",
  "current_site": 5,
  "battery_level": 85,
  "position": {"x": 1.2, "y": 3.4, "yaw": 0.5},
  "velocity": {"linear": 0.0, "angular": 0.0}
}
```

#### 瀵艰埅鍒扮珯鐐?

```http
POST /api/v1/chassis/navigate-to-site
Authorization: Bearer <token>
Content-Type: application/json

{
  "site_id": 5
}
```

#### 閫熷害鎺у埗

```http
POST /api/v1/chassis/velocity
Authorization: Bearer <token>
Content-Type: application/json

{
  "linear_x": 0.5,
  "linear_y": 0.0,
  "angular_z": 0.1
}
```

#### 鍙栨秷/鍋滄瀵艰埅

```http
POST /api/v1/chassis/stop_move
Authorization: Bearer <token>
```

#### 鏆傚仠/鎭㈠瀵艰埅

```http
POST /api/v1/chassis/pause_move
Authorization: Bearer <token>
```

```http
POST /api/v1/chassis/resume_move
Authorization: Bearer <token>
```

---

### 3.6 鑵伴儴鎺у埗

#### 鑾峰彇鑵伴儴鐘舵€?

```http
GET /api/v1/waist/state
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "connected": true,
  "enabled": true,
  "current_position": 230715,
  "current_angle": 0.0,
  "current_speed": 1000,
  "position_reached": true,
  "alarm": false
}
```

#### 鑵伴儴鎺у埗

```http
POST /api/v1/waist/control
Authorization: Bearer <token>
Content-Type: application/json

{
  "command": 5,
  "value": 30.0
}
```

| command | 璇存槑 |
|---------|------|
| 1 | 浣胯兘 |
| 2 | 鍘讳娇鑳?|
| 3 | 璁剧疆閫熷害 |
| 4 | 鍘荤洰鏍囦綅缃?|
| 5 | 鍘荤洰鏍囪搴?(0-45掳) |
| 6 | 鎵嬪姩鍓嶅€?|
| 7 | 鎵嬪姩鍚庝话 |
| 8 | 澶嶄綅鎶ヨ |
| 9 | 鍋滄杩愬姩 |
| 10 | 鍥炲埌绔栫洿 |

---

### 3.7 鍗囬檷鏈烘瀯

#### 鑾峰彇鍗囬檷鐘舵€?

```http
GET /api/v1/lift/state
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "connected": true,
  "enabled": true,
  "current_position": 500.0,
  "current_speed": 20.0,
  "position_reached": true,
  "alarm": false
}
```

#### 鍗囬檷鎺у埗

```http
POST /api/v1/lift/control
Authorization: Bearer <token>
Content-Type: application/json

{
  "command": 4,
  "value": 500.0
}
```

| command | 璇存槑 |
|---------|------|
| 1 | 浣胯兘 |
| 2 | 鍘讳娇鑳?|
| 3 | 璁剧疆閫熷害 |
| 4 | 鍘荤洰鏍囦綅缃?|
| 5 | 鎵嬪姩涓婂崌 |
| 6 | 鎵嬪姩涓嬮檷 |
| 7 | 澶嶄綅鎶ヨ |
| 8 | 鍋滄杩愬姩 |

---

### 3.8 澶撮儴鎺у埗

#### 鑾峰彇澶撮儴鐘舵€?

```http
GET /api/v1/head/state
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "connected": true,
  "pan_position": 500,
  "tilt_position": 500,
  "pan_normalized": 0.0,
  "tilt_normalized": 0.0
}
```

#### 澶撮儴鎺у埗

```http
POST /api/v1/head/control
Authorization: Bearer <token>
Content-Type: application/json

{
  "pan": 0.5,
  "tilt": -0.3,
  "speed": 50.0
}
```

| 鍙傛暟 | 鑼冨洿 | 璇存槑 |
|------|------|------|
| pan | -1.0 ~ 1.0 | 宸﹀彸杞姩 |
| tilt | -1.0 ~ 1.0 | 涓婁笅淇话 |
| speed | 0 ~ 100 | 杩愬姩閫熷害% |

---

### 3.9 浠诲姟绠＄悊

#### 鍒涘缓浠诲姟

```http
POST /api/v1/tasks
Authorization: Bearer <token>
Content-Type: application/json

{
  "name": "鎼繍浠诲姟",
  "description": "浠嶢鐐规惉杩愬埌B鐐?,
  "program": [
    {"action": "navigate", "params": {"site_id": 1}},
    {"action": "arm_move", "params": {"preset": "pick_pose"}},
    {"action": "gripper", "params": {"side": "left", "position": 255}},
    {"action": "navigate", "params": {"site_id": 2}},
    {"action": "gripper", "params": {"side": "left", "position": 0}}
  ]
}
```

#### 鍒楀嚭浠诲姟

```http
GET /api/v1/tasks?status=pending
Authorization: Bearer <token>
```

#### 鍚姩浠诲姟

```http
POST /api/v1/tasks/{task_id}/start
Authorization: Bearer <token>
```

#### 鍙栨秷浠诲姟

```http
POST /api/v1/tasks/{task_id}/cancel
Authorization: Bearer <token>
```

---

### 3.10 绱ф€ュ仠姝?

```http
POST /api/v1/emergency/stop
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "success": true,
  "message": "鎵€鏈夊叧鑺傚凡鍋滄"
}
```

> 鈿狅笍 绱ф€ュ仠姝細绔嬪嵆鍋滄鎵€鏈夎繍鍔紝骞跺己鍒堕噴鏀炬帶鍒舵潈

---

### 3.11 鍔ㄤ綔绠＄悊 (AI 妯″瀷)

#### 鍒楀嚭鎵€鏈夊姩浣?

```http
GET /api/v1/actions
Authorization: Bearer <token>
```

#### 鑾峰彇鍔ㄤ綔璇︽儏

```http
GET /api/v1/actions/{action_id}
Authorization: Bearer <token>
```

#### 鍒涘缓鍔ㄤ綔

```http
POST /api/v1/actions
Authorization: Bearer <token>
Content-Type: application/json

{
  "id": "pickup_cube",
  "name": "澶瑰彇鏂瑰潡",
  "description": "浠庢闈㈠す鍙栨柟鍧?
}
```

---

### 3.12 鏁版嵁褰曞埗

#### 寮€濮嬪綍鍒?

```http
POST /api/v1/recording/start
Authorization: Bearer <token>
Content-Type: application/json

{
  "action_name": "pickup_cube",
  "user_name": "operator1",
  "version": "1.0",
  "topics": [
    "/joint_states",
    "/camera/image_raw"
  ]
}
```

#### 鍋滄褰曞埗

```http
POST /api/v1/recording/stop
Authorization: Bearer <token>
```

#### 褰曞埗鐘舵€?

```http
GET /api/v1/recording/status
Authorization: Bearer <token>
```

---

### 3.13 鐩告満瑙嗛娴?

#### 鑾峰彇鐩告満鍒楄〃/鐘舵€?

```http
GET /api/v1/camera/status
```

**鍝嶅簲:**
```json
{
  "web_video_server_available": true,
  "cameras": {
    "head": {"topic": "/head_camera/color/image_raw", "available": true},
    "left_hand": {"topic": "/left_camera/color/image_raw", "available": true},
    "right_hand": {"topic": "/right_camera/color/image_raw", "available": true}
  }
}
```

#### 鑾峰彇瑙嗛娴?(MJPEG)

```http
GET /api/v1/camera/stream/{camera_id}?quality=50&width=640&height=480
```

| 鍙傛暟 | 绫诲瀷 | 璇存槑 |
|------|------|------|
| camera_id | path | head, left_hand, right_hand |
| quality | query | JPEG 璐ㄩ噺 1-100 |
| width | query | 杈撳嚭瀹藉害 (鍙€? |
| height | query | 杈撳嚭楂樺害 (鍙€? |

**鍝嶅簲**: `multipart/x-mixed-replace` MJPEG 娴?

#### 鑾峰彇鍗曞抚蹇収

```http
GET /api/v1/camera/snapshot/{camera_id}?quality=80
```

**鍝嶅簲**: `image/jpeg`

---

### 3.14 棰勮绠＄悊

#### 鑾峰彇棰勮鍒楄〃

```http
GET /api/v1/presets/{preset_type}?category=custom&include_builtin=true
Authorization: Bearer <token>
```

| preset_type | 璇存槑 |
|-------------|------|
| location | 浣嶇疆鐐?|
| arm_pose | 鏈烘鑷傚Э鎬?|
| lift_height | 鍗囬檷楂樺害 |
| head_position | 澶撮儴浣嶇疆 |
| gripper_position | 澶圭埅浣嶇疆 |
| task_template | 浠诲姟妯℃澘 |

**鍝嶅簲:**
```json
{
  "type": "arm_pose",
  "total": 5,
  "items": [
    {
      "id": "home",
      "name": "鍒濆浣嶇疆",
      "description": "鏈烘鑷傚綊闆朵綅缃?,
      "category": "builtin",
      "left_joints": [0, 0, 0, 0, 0, 0, 0],
      "right_joints": [0, 0, 0, 0, 0, 0, 0]
    }
  ]
}
```

#### 鍒涘缓棰勮

```http
POST /api/v1/presets/{preset_type}
Authorization: Bearer <token>
Content-Type: application/json

{
  "name": "鎶撳彇浣嶇疆",
  "description": "妗岄潰鎶撳彇濮挎€?,
  "category": "custom",
  "data": {
    "left_joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
    "right_joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
  }
}
```

#### 鏇存柊棰勮

```http
PUT /api/v1/presets/{preset_type}/{preset_id}
Authorization: Bearer <token>
```

#### 鍒犻櫎棰勮

```http
DELETE /api/v1/presets/{preset_type}/{preset_id}
Authorization: Bearer <token>
```

#### 鎵ц棰勮

```http
POST /api/v1/presets/{preset_type}/{preset_id}/execute
Authorization: Bearer <token>
```

---

### 3.15 VR 閬ユ搷浣?

#### 鑾峰彇 VR 绯荤粺鐘舵€?

```http
GET /api/v1/vr/status
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "connected": true,
  "head_pose": {
    "position": [0.0, 1.5, 0.0],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "left_controller": {
    "active": true,
    "pose": {"position": [-0.3, 1.0, 0.3], "orientation": [0, 0, 0, 1]},
    "joystick": [0.0, 0.0],
    "trigger": 0.0,
    "grip": 0.8,
    "buttons": [0, 0, 0, 0],
    "clutch_engaged": true
  },
  "right_controller": {
    "active": true,
    "pose": {"position": [0.3, 1.0, 0.3], "orientation": [0, 0, 0, 1]},
    "joystick": [0.0, 0.0],
    "trigger": 0.5,
    "grip": 0.0,
    "buttons": [0, 0, 0, 0],
    "clutch_engaged": false
  },
  "clutch": {
    "left_clutch_engaged": true,
    "right_clutch_engaged": false,
    "left_grip_value": 0.8,
    "right_grip_value": 0.0
  }
}
```

#### 鑾峰彇 Clutch 鐘舵€?

```http
GET /api/v1/vr/clutch
Authorization: Bearer <token>
```

**鍝嶅簲:**
```json
{
  "left_clutch_engaged": true,
  "right_clutch_engaged": false,
  "left_grip_value": 0.8,
  "right_grip_value": 0.0
}
```

---

 # QYH Jushen Robot API 接口文档

> **版本**: 2.0.0  
> **更新日期**: 2026-01-15  
> **基础URL**: `http://<robot_ip>:8000/api/v1`
> 
> **⚠️ 重要更新**: v2.0 版本统一了所有 API 路由前缀为 `/api/v1`，并采用统一响应格式。旧路由 `/api/*` 仍然可用但已标记为 deprecated。

---

## 目录

1. [概述](#1-概述)
2. [统一响应格式](#2-统一响应格式)
3. [认证机制](#3-认证机制)
4. [REST API 接口](#4-rest-api-接口)
5. [WebSocket 接口](#5-websocket-接口)
6. [错误码说明](#6-错误码说明)
7. [行业最佳实践](#7-行业最佳实践)
8. [SDK 集成示例](#8-sdk-集成示例)

---

## 1. 概述

### 1.1 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                     外部系统 / 客户端                         │
│  (Web前端、移动App、MES系统、第三方集成)                       │
└─────────────────────────────┬───────────────────────────────┘
                              │
                    ┌─────────▼─────────┐
                    │   HTTP/WebSocket   │
                    │   (对外网关层)      │
                    └─────────┬─────────┘
                              │
┌─────────────────────────────▼───────────────────────────────┐
│                    QYH Jushen Backend                        │
│  ┌──────────────┬──────────────┬──────────────────────────┐ │
│  │  REST API    │  WebSocket   │  安全/控制权管理           │ │
│  │  (控制/配置)  │  (实时推送)   │  (认证、急停、看门狗)      │ │
│  └──────┬───────┴──────┬───────┴─────────────┬────────────┘ │
│         │              │                     │              │
│         └──────────────┼─────────────────────┘              │
│                        │                                    │
│              ┌─────────▼─────────┐                          │
│              │   ROS2 Bridge     │                          │
│              │   (内部通信)       │                          │
│              └─────────┬─────────┘                          │
└────────────────────────┼────────────────────────────────────┘
                         │
           ┌─────────────▼─────────────┐
           │   ROS2 / 硬件驱动层        │
           │   (底盘、机械臂、传感器)    │
           └───────────────────────────┘
```

### 1.2 接口协议

| 协议 | 用途 | 特点 |
|------|------|------|
| **REST/HTTP** | 控制命令、配置、任务管理 | 请求-响应模式，适合低频操作 |
| **WebSocket** | 实时状态推送、遥操作 | 双向通信，30Hz 状态推送 |

### 1.3 API 文档

本后端集成了 **Swagger UI** 自动文档：
- Swagger UI: `http://<robot_ip>:8000/docs`
- ReDoc: `http://<robot_ip>:8000/redoc`
- OpenAPI JSON: `http://<robot_ip>:8000/openapi.json`

### 1.4 API 版本

| 版本 | 路由前缀 | 状态 |
|------|----------|------|
| v1 (推荐) | `/api/v1/*` | 当前版本 |
| legacy | `/api/*` | deprecated，将在 v3 移除 |

---

## 2. 统一响应格式

### 2.1 成功响应

```json
{
  "success": true,
  "code": 0,
  "message": "操作成功",
  "data": { ... },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

### 2.2 错误响应

```json
{
  "success": false,
  "code": 1101,
  "message": "用户名或密码错误",
  "data": null,
  "timestamp": "2026-01-15T10:30:00.000Z",
  "error": {
    "code": "AUTH_INVALID_CREDENTIALS",
    "message": "用户名或密码错误",
    "field": null,
    "details": null
  }
}
```

### 2.3 分页响应

```json
{
  "success": true,
  "code": 0,
  "message": "",
  "data": {
    "items": [...],
    "total": 100
  },
  "timestamp": "2026-01-15T10:30:00.000Z"
}
```

---

## 3. 认证机制

### 3.1 JWT Token 认证

#### 登录

```http
POST /api/v1/auth/login
Content-Type: application/json

{
  "username": "admin",
  "password": "admin123"
}
```

#### 刷新 Token

```http
POST /api/v1/auth/refresh
Authorization: Bearer <current_token>
```

---

## 4. REST API 接口（节选）

### 4.1 系统状态

- `GET /health` 健康检查
- `GET /api/v1` API 版本信息
- `GET /api/v1/robot/status` 机器人状态

### 4.2 控制权管理

- `POST /api/v1/control/acquire`
- `POST /api/v1/control/release`
- `POST /api/v1/control/renew`
- `GET /api/v1/control/status`

### 4.3 机械臂/夹爪/底盘/升降/腰部/头部

- 机械臂: `/api/v1/arm/*`
- 夹爪: `/api/v1/gripper/*`
- 底盘: `/api/v1/chassis/*`
- 升降: `/api/v1/lift/*`
- 腰部: `/api/v1/waist/*`
- 头部: `/api/v1/head/*`

### 4.4 任务与动作

- 任务: `/api/v1/tasks/*`
- 动作: `/api/v1/actions/*`

### 4.5 录制/相机/预设/VR/LED

- 录制: `/api/v1/recording/*`
- 相机: `/api/v1/camera/*`
- 预设: `/api/v1/presets/*`
- VR: `/api/v1/vr/*`
- LED灯带: `/api/v1/led/*`

### 4.6 LED灯带控制 API

LED灯带与夹爪共用RS-485总线，通过gripper_control_node控制。

#### 4.6.1 获取LED状态

```http
GET /api/v1/led/state
Authorization: Bearer <token>
```

**响应:**
```json
{
  "success": true,
  "code": 0,
  "data": {
    "is_blinking": false,
    "current_color": {"r": 0, "g": 255, "b": 0, "w": 0},
    "blink_colors": null,
    "blink_interval_ms": null
  }
}
```

#### 4.6.2 设置LED纯色

```http
POST /api/v1/led/color
Authorization: Bearer <token>
Content-Type: application/json

{
  "r": 255,
  "g": 0,
  "b": 0,
  "w": 0
}
```

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| r | int | 0-255 | 红色 |
| g | int | 0-255 | 绿色 |
| b | int | 0-255 | 蓝色 |
| w | int | 0-255 | 白色(W通道，产生更纯净白光) |

#### 4.6.3 设置预设颜色

```http
POST /api/v1/led/preset/{preset_name}
Authorization: Bearer <token>
```

**可用预设:**
| 预设名 | 颜色 | RGBW值 |
|--------|------|--------|
| red | 红色 | (255,0,0,0) |
| green | 绿色 | (0,255,0,0) |
| blue | 蓝色 | (0,0,255,0) |
| white | 纯白(W通道) | (0,0,0,255) |
| white_rgb | RGB白色 | (255,255,255,0) |
| yellow | 黄色 | (255,255,0,0) |
| cyan | 青色 | (0,255,255,0) |
| magenta | 品红 | (255,0,255,0) |
| orange | 橙色 | (255,128,0,0) |
| purple | 紫色 | (128,0,255,0) |
| warm_white | 暖白 | (50,30,0,200) |
| off | 关闭 | (0,0,0,0) |

#### 4.6.4 设置自定义闪烁

```http
POST /api/v1/led/blink
Authorization: Bearer <token>
Content-Type: application/json

{
  "colors": [
    {"r": 255, "g": 0, "b": 0, "w": 0},
    {"r": 0, "g": 255, "b": 0, "w": 0}
  ],
  "interval_ms": 500
}
```

| 参数 | 类型 | 说明 |
|------|------|------|
| colors | array | 颜色序列(至少1个) |
| interval_ms | int | 切换间隔(50-10000ms) |

#### 4.6.5 设置预设闪烁模式

```http
POST /api/v1/led/blink/preset/{mode_name}
Authorization: Bearer <token>
```

**可用模式:**
| 模式 | 说明 | 颜色 | 间隔 |
|------|------|------|------|
| warning | 黄色警告 | 黄/黑 | 200ms |
| error | 红色错误 | 红/黑 | 300ms |
| success | 绿色成功 | 绿/黑 | 500ms |
| processing | 处理中 | 蓝/青 | 400ms |
| rainbow | 彩虹 | 7色循环 | 300ms |
| police | 警灯 | 红/蓝 | 150ms |

#### 4.6.6 停止闪烁

```http
POST /api/v1/led/stop
Authorization: Bearer <token>
```

停止闪烁并恢复为节点配置的默认颜色。

#### 4.6.7 获取所有预设

```http
GET /api/v1/led/presets
Authorization: Bearer <token>
```

返回所有可用的预设颜色和闪烁模式。

---

## 5. WebSocket 接口

- `ws://<robot_ip>:8000/ws?token=<jwt_token>`
- `ws://<robot_ip>:8000/ws/robot`

---

## 6. 错误码说明

详见统一响应中的 `code` 及 `error.code` 字段。

---

## 7. 行业最佳实践

建议业务系统使用 REST 下发任务，WebSocket 监听状态与任务更新。

---

## 8. SDK 集成示例

请参考 Swagger UI 或后续 SDK 文档。

| type | 璇存槑 | 棰戠巼 |
|------|------|------|
| `robot_state` | 鏈哄櫒浜哄畬鏁寸姸鎬?| 30Hz |
| `error` | 閿欒閫氱煡 | 浜嬩欢瑙﹀彂 |
| `task_update` | 浠诲姟鐘舵€佹洿鏂?| 浜嬩欢瑙﹀彂 |
| `control_change` | 鎺у埗鏉冨彉鏇撮€氱煡 | 浜嬩欢瑙﹀彂 |

#### robot_state 娑堟伅鏍煎紡

```json
{
  "type": "robot_state",
  "data": {
    "timestamp": "2026-01-15T10:30:00.123Z",
    "joints": {
      "left_arm": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
      "right_arm": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    },
    "grippers": {
      "left": {"position": 100, "force": 50},
      "right": {"position": 0, "force": 0}
    },
    "base": {
      "x": 1.2, "y": 3.4, "theta": 0.5,
      "velocity": {"linear": 0.3, "angular": 0.1}
    }
  }
}
```

### 4.3 瀹㈡埛绔彂閫佹秷鎭被鍨?

| type | 璇存槑 |
|------|------|
| `heartbeat` | 蹇冭烦淇濇椿 |
| `subscribe` | 璁㈤槄璇濋 |
| `velocity_command` | 閫熷害鎸囦护 (閬ユ搷浣? |

#### 鍙戦€侀€熷害鎸囦护

```json
{
  "type": "velocity_command",
  "data": {
    "linear_x": 0.5,
    "linear_y": 0.0,
    "angular_z": 0.1
  }
}
```

### 4.4 3D 鍙鍖?WebSocket

**杩炴帴鍦板潃:** `ws://<robot_ip>:8000/ws/robot` (鏃犻渶璁よ瘉锛岀敤浜庢紨绀?

鐢ㄤ簬 3D 妯″瀷鍙鍖栫殑鍏宠妭鐘舵€佹帹閫侊細

```json
{
  "type": "joint_states",
  "data": {
    "name": ["joint1", "joint2", "joint3", ...],
    "position": [0.1, 0.2, 0.3, ...],
    "velocity": [0.0, 0.0, 0.0, ...],
    "effort": [0.0, 0.0, 0.0, ...]
  }
}
```

---

## 5. 閿欒鐮佽鏄?

### 5.1 HTTP 鐘舵€佺爜

| 鐘舵€佺爜 | 璇存槑 |
|--------|------|
| 200 | 鎴愬姛 |
| 201 | 鍒涘缓鎴愬姛 |
| 400 | 璇锋眰鍙傛暟閿欒 |
| 401 | 鏈璇?/ Token 鏃犳晥 |
| 403 | 鏉冮檺涓嶈冻 |
| 404 | 璧勬簮涓嶅瓨鍦?|
| 409 | 鍐茬獊 (濡傛帶鍒舵潈宸茶鍗犵敤) |
| 500 | 鏈嶅姟鍣ㄥ唴閮ㄩ敊璇?|
| 503 | 鏈嶅姟涓嶅彲鐢?(濡?ROS2 鏈繛鎺? |

### 5.2 閿欒鍝嶅簲鏍煎紡

```json
{
  "detail": "鎺у埗鏉冨凡琚敤鎴?admin 鎸佹湁"
}
```

---

## 6. 琛屼笟鏈€浣冲疄璺?

### 6.1 鎺ュ彛璁捐鍘熷垯

| 鍘熷垯 | 璇存槑 |
|------|------|
| REST + WebSocket | REST 鐢ㄤ簬鎺у埗/閰嶇疆锛學ebSocket 鐢ㄤ簬瀹炴椂鐘舵€?|
| 鎺у埗鏉冮攣 | 闃叉澶氬鎴风鍐茬獊鎺у埗鏈哄櫒浜?|
| 蹇冭烦鏈哄埗 | 妫€娴嬭繛鎺ュ瓨娲伙紝鑷姩閲婃斁鎺у埗鏉?|
| 鐪嬮棬鐙?| 瀹㈡埛绔柇杩炲悗鑷姩鍋滄鏈哄櫒浜?|

### 6.2 涓庝富娴佸巶鍟嗗姣?

| 鍘傚晢 | 鎺ュ彛鏂规 |
|------|----------|
| **Universal Robots (UR)** | REST API + RTDE (瀹炴椂鏁版嵁) |
| **ABB** | Robot Web Services (REST) + WebSocket |
| **FANUC** | FANUC Web Server (REST) |
| **Boston Dynamics** | REST API + gRPC |
| **QYH (鏈郴缁?** | REST API + WebSocket 鉁?|

### 6.3 闆嗘垚寤鸿

```
澶栭儴绯荤粺闆嗘垚寤鸿锛?

1. MES/WMS 绯荤粺闆嗘垚
   - 浣跨敤 REST API 涓嬪彂浠诲姟
   - WebSocket 鐩戝惉浠诲姟鐘舵€佸彉鍖?

2. 瑙嗚/AI 绯荤粺闆嗘垚
   - REST API 鑾峰彇鐩告満瑙嗛娴?
   - REST API 瑙﹀彂鍔ㄤ綔鎵ц

3. 鐩戞帶绯荤粺闆嗘垚
   - WebSocket 瀹炴椂鑾峰彇鐘舵€?
   - REST API 鑾峰彇鍘嗗彶鏁版嵁

4. 绉诲姩绔?App
   - REST API 鐧诲綍璁よ瘉
   - WebSocket 瀹炴椂閬ユ搷浣?
```

---

## 7. SDK 闆嗘垚绀轰緥

### 7.1 Python SDK 绀轰緥

```python
import requests
import websockets
import asyncio
import json

class QYHRobotClient:
    """QYH 鏈哄櫒浜?Python SDK"""
    
    def __init__(self, host: str, port: int = 8000):
        self.base_url = f"http://{host}:{port}"
        self.ws_url = f"ws://{host}:{port}/ws"
        self.token = None
    
    def login(self, username: str, password: str) -> bool:
        """鐧诲綍鑾峰彇 Token"""
        resp = requests.post(
            f"{self.base_url}/api/auth/login",
            json={"username": username, "password": password}
        )
        if resp.status_code == 200:
            self.token = resp.json()["access_token"]
            return True
        return False
    
    @property
    def headers(self):
        return {"Authorization": f"Bearer {self.token}"}
    
    def get_status(self) -> dict:
        """鑾峰彇鏈哄櫒浜虹姸鎬?""
        resp = requests.get(
            f"{self.base_url}/api/robot/status",
            headers=self.headers
        )
        return resp.json()
    
    def acquire_control(self, duration: int = 300) -> bool:
        """鑾峰彇鎺у埗鏉?""
        resp = requests.post(
            f"{self.base_url}/api/control/acquire",
            headers=self.headers,
            json={"duration": duration}
        )
        return resp.status_code == 200
    
    def release_control(self) -> bool:
        """閲婃斁鎺у埗鏉?""
        resp = requests.post(
            f"{self.base_url}/api/control/release",
            headers=self.headers
        )
        return resp.status_code == 200
    
    def emergency_stop(self) -> bool:
        """绱ф€ュ仠姝?""
        resp = requests.post(
            f"{self.base_url}/api/v1/emergency/stop",
            headers=self.headers
        )
        return resp.status_code == 200
    
    def move_arm_joint(self, positions: list, velocity: float = 0.5) -> dict:
        """鍏宠妭杩愬姩"""
        resp = requests.post(
            f"{self.base_url}/api/v1/arm/movej",
            headers=self.headers,
            json={
                "robot_id": -1,
                "joint_positions": positions,
                "velocity": velocity,
                "is_block": True
            }
        )
        return resp.json()
    
    def control_gripper(self, side: str, position: int) -> dict:
        """鎺у埗澶圭埅"""
        resp = requests.post(
            f"{self.base_url}/api/v1/gripper/move",
            headers=self.headers,
            json={"side": side, "position": position}
        )
        return resp.json()
    
    async def subscribe_state(self, callback):
        """璁㈤槄瀹炴椂鐘舵€?""
        uri = f"{self.ws_url}?token={self.token}"
        async with websockets.connect(uri) as ws:
            while True:
                msg = await ws.recv()
                data = json.loads(msg)
                if data["type"] == "robot_state":
                    callback(data["data"])


# 浣跨敤绀轰緥
if __name__ == "__main__":
    robot = QYHRobotClient("192.168.1.100")
    
    # 鐧诲綍
    robot.login("admin", "admin123")
    
    # 鑾峰彇鐘舵€?
    status = robot.get_status()
    print(f"鐢垫睜: {status['system']['battery']}%")
    
    # 鑾峰彇鎺у埗鏉?
    if robot.acquire_control():
        # 鎺у埗澶圭埅
        robot.control_gripper("left", 255)  # 鍏抽棴
        robot.control_gripper("left", 0)    # 鎵撳紑
        
        # 閲婃斁鎺у埗鏉?
        robot.release_control()
```

### 7.2 JavaScript/TypeScript SDK 绀轰緥

```typescript
class QYHRobotClient {
  private baseUrl: string;
  private wsUrl: string;
  private token: string | null = null;
  private ws: WebSocket | null = null;

  constructor(host: string, port: number = 8000) {
    this.baseUrl = `http://${host}:${port}`;
    this.wsUrl = `ws://${host}:${port}/ws`;
  }

  async login(username: string, password: string): Promise<boolean> {
    const resp = await fetch(`${this.baseUrl}/api/auth/login`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ username, password })
    });
    
    if (resp.ok) {
      const data = await resp.json();
      this.token = data.access_token;
      return true;
    }
    return false;
  }

  private get headers() {
    return {
      'Authorization': `Bearer ${this.token}`,
      'Content-Type': 'application/json'
    };
  }

  async getStatus(): Promise<any> {
    const resp = await fetch(`${this.baseUrl}/api/robot/status`, {
      headers: this.headers
    });
    return resp.json();
  }

  async acquireControl(duration: number = 300): Promise<boolean> {
    const resp = await fetch(`${this.baseUrl}/api/control/acquire`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({ duration })
    });
    return resp.ok;
  }

  async emergencyStop(): Promise<boolean> {
    const resp = await fetch(`${this.baseUrl}/api/v1/emergency/stop`, {
      method: 'POST',
      headers: this.headers
    });
    return resp.ok;
  }

  subscribeState(callback: (state: any) => void): void {
    this.ws = new WebSocket(`${this.wsUrl}?token=${this.token}`);
    
    this.ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === 'robot_state') {
        callback(data.data);
      }
    };

    // 蹇冭烦淇濇椿
    setInterval(() => {
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify({ type: 'heartbeat' }));
      }
    }, 5000);
  }

  disconnect(): void {
    this.ws?.close();
  }
}

// 浣跨敤绀轰緥
const robot = new QYHRobotClient('192.168.1.100');

(async () => {
  await robot.login('admin', 'admin123');
  
  const status = await robot.getStatus();
  console.log(`鐢垫睜: ${status.system.battery}%`);
  
  // 璁㈤槄瀹炴椂鐘舵€?
  robot.subscribeState((state) => {
    console.log('瀹炴椂鐘舵€?', state);
  });
})();
```

---

## 闄勫綍

### A. 瀹屾暣鎺ュ彛鍒楄〃

| 鏂规硶 | 璺緞 | 璇存槑 | 鏉冮檺 |
|------|------|------|------|
| **璁よ瘉** |
| POST | `/api/auth/login` | 鐧诲綍 | 鏃?|
| POST | `/api/auth/logout` | 鐧诲嚭 | 鏃?|
| POST | `/api/auth/refresh` | 鍒锋柊 Token | 闇€璁よ瘉 |
| **鏈哄櫒浜虹姸鎬?* |
| GET | `/api/robot/status` | 鏈哄櫒浜虹姸鎬?| operator |
| GET | `/health` | 鍋ュ悍妫€鏌?| 鏃?|
| **鎺у埗鏉?* |
| POST | `/api/control/acquire` | 鑾峰彇鎺у埗鏉?| operator |
| POST | `/api/control/release` | 閲婃斁鎺у埗鏉?| operator |
| GET | `/api/control/status` | 鎺у埗鏉冪姸鎬?| 鏃?|
| **绱ф€ユ帶鍒?* |
| POST | `/api/v1/emergency/stop` | 绱ф€ュ仠姝?| operator |
| **鏈烘鑷?* |
| GET | `/api/v1/arm/state` | 鏈烘鑷傜姸鎬?| admin |
| GET | `/api/v1/arm/servo/status` | 浼烘湇鐘舵€?| admin |
| POST | `/api/v1/arm/connect` | 杩炴帴 | admin |
| POST | `/api/v1/arm/disconnect` | 鏂紑 | admin |
| POST | `/api/v1/arm/power_on` | 涓婄數 | admin |
| POST | `/api/v1/arm/power_off` | 涓嬬數 | admin |
| POST | `/api/v1/arm/enable` | 浣胯兘 | admin |
| POST | `/api/v1/arm/disable` | 鍘讳娇鑳?| admin |
| POST | `/api/v1/arm/clear_error` | 娓呴櫎閿欒 | admin |
| POST | `/api/v1/arm/movej` | 鍏宠妭杩愬姩 | admin |
| POST | `/api/v1/arm/movel` | 鐩寸嚎杩愬姩 | admin |
| POST | `/api/v1/arm/jog` | 鐐瑰姩鎺у埗 | admin |
| POST | `/api/v1/arm/jog_stop` | 鍋滄鐐瑰姩 | admin |
| **澶圭埅** |
| GET | `/api/v1/gripper/state` | 鍙屽す鐖姸鎬?| operator |
| GET | `/api/v1/gripper/{side}/state` | 鍗曞す鐖姸鎬?| operator |
| POST | `/api/v1/gripper/{side}/enable` | 浣胯兘澶圭埅 | operator |
| POST | `/api/v1/gripper/activate` | 婵€娲诲す鐖?| operator |
| POST | `/api/v1/gripper/move` | 鎺у埗澶圭埅 | operator |
| **搴曠洏** |
| GET | `/api/v1/chassis/status` | 搴曠洏鐘舵€?| operator |
| GET | `/api/v1/chassis/navigation_status` | 瀵艰埅鐘舵€?| operator |
| POST | `/api/v1/chassis/velocity` | 閫熷害鎺у埗 | operator |
| POST | `/api/v1/chassis/stop` | 鍋滄 | operator |
| POST | `/api/v1/chassis/navigate-to-site` | 瀵艰埅鍒扮珯鐐?| operator |
| POST | `/api/v1/chassis/stop_move` | 鍋滄绉诲姩 | operator |
| POST | `/api/v1/chassis/pause_move` | 鏆傚仠绉诲姩 | operator |
| POST | `/api/v1/chassis/resume_move` | 鎭㈠绉诲姩 | operator |
| POST | `/api/v1/chassis/manual_mode` | 鎵嬪姩妯″紡 | operator |
| **鍗囬檷** |
| GET | `/api/v1/lift/state` | 鍗囬檷鐘舵€?| admin |
| POST | `/api/v1/lift/control` | 鍗囬檷鎺у埗 | admin |
| POST | `/api/v1/lift/enable` | 鍗囬檷浣胯兘 | admin |
| **鑵伴儴** |
| GET | `/api/v1/waist/state` | 鑵伴儴鐘舵€?| admin |
| POST | `/api/v1/waist/control` | 鑵伴儴鎺у埗 | admin |
| **澶撮儴** |
| GET | `/api/v1/head/state` | 澶撮儴鐘舵€?| admin |
| POST | `/api/v1/head/control` | 澶撮儴鎺у埗 | admin |
| POST | `/api/v1/head/enable` | 澶撮儴浣胯兘 | admin |
| POST | `/api/v1/head/disable` | 澶撮儴鍘讳娇鑳?| admin |
| **浠诲姟** |
| GET | `/api/v1/tasks` | 浠诲姟鍒楄〃 | user |
| GET | `/api/v1/tasks/{id}` | 浠诲姟璇︽儏 | user |
| POST | `/api/v1/tasks` | 鍒涘缓浠诲姟 | operator |
| POST | `/api/v1/tasks/{id}/start` | 鍚姩浠诲姟 | operator |
| POST | `/api/v1/tasks/{id}/cancel` | 鍙栨秷浠诲姟 | operator |
| **鍔ㄤ綔** |
| GET | `/api/v1/actions` | 鍔ㄤ綔鍒楄〃 | user |
| GET | `/api/v1/actions/{id}` | 鍔ㄤ綔璇︽儏 | user |
| POST | `/api/v1/actions` | 鍒涘缓鍔ㄤ綔 | admin |
| DELETE | `/api/v1/actions/{id}` | 鍒犻櫎鍔ㄤ綔 | admin |
| **褰曞埗** |
| GET | `/api/v1/recording/status` | 褰曞埗鐘舵€?| operator |
| POST | `/api/v1/recording/start` | 寮€濮嬪綍鍒?| operator |
| POST | `/api/v1/recording/stop` | 鍋滄褰曞埗 | operator |
| **鐩告満** |
| GET | `/api/v1/camera/status` | 鐩告満鐘舵€?| 鏃?|
| GET | `/api/v1/camera/stream/{id}` | 瑙嗛娴?| 鏃?|
| GET | `/api/v1/camera/snapshot/{id}` | 蹇収 | 鏃?|
| **棰勮** |
| GET | `/api/v1/presets/{type}` | 棰勮鍒楄〃 | admin |
| GET | `/api/v1/presets/{type}/{id}` | 棰勮璇︽儏 | admin |
| POST | `/api/v1/presets/{type}` | 鍒涘缓棰勮 | admin |
| PUT | `/api/v1/presets/{type}/{id}` | 鏇存柊棰勮 | admin |
| DELETE | `/api/v1/presets/{type}/{id}` | 鍒犻櫎棰勮 | admin |
| POST | `/api/v1/presets/{type}/{id}/execute` | 鎵ц棰勮 | admin |
| **VR 閬ユ搷浣?* |
| GET | `/api/v1/vr/status` | VR 鐘舵€?| admin |
| GET | `/api/v1/vr/clutch` | Clutch 鐘舵€?| admin |
| **WebSocket** |
| WS | `/ws?token=<token>` | 涓?WebSocket | 闇€璁よ瘉 |
| WS | `/ws/robot` | 3D 鍙鍖?| 鏃?|

### B. 鑱旂郴鏂瑰紡

- **鎶€鏈敮鎸?*: support@qyh-robot.com
- **API 闂**: api@qyh-robot.com
