// Calibration Tab Module
window.CameraApp = window.CameraApp || {};

window.CameraApp.CalibrationTab = {
    previewTimer: null,
    statusTimer: null,
    paramsSaveTimeout: null,
    currentCalibration: null,
    monoCaptureTimer: null,
    activeMonoJobId: null,
    activeMonoCameraId: null,
    livePreviewElement: null,
    livePreviewPlaceholder: null,
    autoComputeEnabled: false,
    autoComputeTriggered: false,

    selectedCameras: new Set(),
    captureInProgress: false,
    calibrationStatus: null,
    calibrationParams: null,

    async init() {
        this.selectedCameras = new Set();
        this.captureInProgress = false;
        this.calibrationStatus = null;
        this.calibrationParams = null;
        this.monoCaptureTimer = null;
        this.activeMonoJobId = null;
        this.activeMonoCameraId = null;
        this.livePreviewElement = null;
        this.livePreviewPlaceholder = null;
        this.autoComputeEnabled = false;
        this.autoComputeTriggered = false;
        this.setupEventListeners();
        await this.loadCalibrationParams();
        await this.refreshCalibrationStatus();
    },

    setupEventListeners() {
        // Event delegation for dynamic elements
        document.addEventListener('click', this.handleDynamicEvents.bind(this));
        document.addEventListener('change', this.handleDynamicChanges.bind(this));
    },

    handleDynamicEvents(event) {
        const button = event.target.closest('button');

        if (!button) {
            return;
        }

        switch (button.id) {
            case 'start-mono-capture-btn':
                this.startMonoCapture();
                break;
            case 'start-stereo-capture-btn':
                this.startStereoCapture();
                break;
            case 'start-calibration-btn':
                this.startAutoCalibration();
                break;
            case 'stop-calibration-btn':
                this.stopAutoCalibration();
                break;
            case 'compute-calibration-btn':
                this.computeCalibrationResults();
                break;
            case 'save-params-btn':
                this.saveCalibrationParams();
                break;
            case 'reset-params-btn':
                this.resetCalibrationParams();
                break;
            default:
                break;
        }
    },

    handleDynamicChanges(event) {
        // Handle parameter changes
        if (event.target.matches('#calibration-parameters input, #calibration-parameters select')) {
            // Auto-save parameters after a short delay
            clearTimeout(this.paramsSaveTimeout);
            this.paramsSaveTimeout = setTimeout(() => {
                this.saveCalibrationParams();
            }, 1000);
        }


        if (event.target.matches('.calibration-camera-select')) {
            const cameraId = event.target.dataset.cameraId;

            if (!cameraId) {
                return;
            }

            if (event.target.checked) {
                this.selectedCameras.add(cameraId);
            } else {
                this.selectedCameras.delete(cameraId);
            }

            this.updateCaptureControls();
        }
    },

    async loadCalibrationParams() {
        try {
            const params = await window.CameraApp.API.getCalibrationParams();
            this.calibrationParams = params;
            this.renderCalibrationInterface(params);
        } catch (error) {
            console.error('Failed to load calibration parameters:', error);
            this.calibrationParams = window.CameraApp.Config.CALIBRATION;
            this.renderCalibrationInterface(this.calibrationParams);
        }
    },

    renderCalibrationInterface(params) {
        // Render main calibration content
        const content = document.getElementById('calibration-content');
        content.innerHTML = `
            <div class="calibration-step">
                <h6 class="fw-bold mb-3">
                    <i class="bi bi-1-circle me-2"></i>Capture Calibration Frames
                </h6>
                <p class="text-muted mb-3">
                    Capture calibration frames directly from the live camera feeds. Select one camera for mono capture or
                    two cameras for stereo capture, adjust optional frame limits, and keep the checkerboard pattern visible
                    throughout the sequence.                </p>


                <div class="row g-3 align-items-end mb-3">
                    <div class="col-sm-6 col-md-3">
                        <label class="form-label small mb-1">Mono frame count (optional)</label>
                        <input type="number" class="form-control form-control-sm" id="mono-frame-count" min="1" placeholder="Auto">
                    </div>
                    <div class="col-sm-6 col-md-3">
                        <label class="form-label small mb-1">Mono interval (ms)</label>
                        <input type="number" class="form-control form-control-sm" id="mono-frame-interval" min="0" placeholder="Auto">
                        <small class="form-text">Delay between captured mono frames</small>
                    </div>
                    <div class="col-sm-6 col-md-3">
                        <label class="form-label small mb-1">Stereo frame count (optional)</label>
                        <input type="number" class="form-control form-control-sm" id="stereo-frame-count" min="1" placeholder="Auto">
                    </div>
                    <div class="col-sm-6 col-md-3">
                        <label class="form-label small mb-1">Stereo interval (ms)</label>
                        <input type="number" class="form-control form-control-sm" id="stereo-frame-interval" min="0" placeholder="Auto">
                        <small class="form-text">Delay between each stereo capture</small>
                    </div>
                </div>


                <div class="row g-3 mb-3">
                    <div class="col-md-6">
                        <button class="btn btn-success w-100" id="start-mono-capture-btn">
                            <i class="bi bi-camera me-1"></i>Start Mono Capture
                        </button>
                    </div>
                    <div class="col-md-6">
                        <button class="btn btn-primary w-100" id="start-stereo-capture-btn">
                            <i class="bi bi-camera-video me-1"></i>Start Stereo Capture
                        </button>
                    </div>
                </div>

                <div id="capture-status" class="calibration-status" style="display: none;"></div>
                <div id="mono-progress-panel" class="card shadow-sm border-0 mb-3" style="display: none;">
                    <div class="card-body">
                        <div class="d-flex justify-content-between align-items-center mb-2">
                            <h6 class="mb-0 text-uppercase small text-muted">Mono capture progress</h6>
                            <span class="badge bg-secondary" id="mono-progress-visibility" style="display: none;">Initializing</span>
                        </div>
                        <div class="progress mb-2">
                            <div class="progress-bar" id="mono-progress-bar" role="progressbar" style="width: 0%" aria-valuemin="0" aria-valuemax="100">0%</div>
                        </div>
                        <div class="d-flex justify-content-between small text-muted mb-3">
                            <span id="mono-progress-count">0 / 0 frames</span>
                            <span id="mono-progress-hint">Waiting for checkerboard...</span>
                        </div>
                        <div id="mono-progress-preview-wrapper" class="text-center" style="display: none;">
                            <img id="mono-progress-preview" class="img-fluid rounded border" alt="Mono capture preview" style="max-height: 260px;">
                        </div>
                        <div id="mono-progress-summary" class="small text-muted mt-3" style="display: none;"></div>
                    </div>
                </div>
                <div id="capture-summary" class="small text-muted"></div>
            </div>

            <div class="calibration-step">
                <h6 class="fw-bold mb-3">
                    <i class="bi bi-2-circle me-2"></i>Process & Calibrate
                </h6>
                <p class="text-muted mb-3">
                    Automatically process the captured images and run calibration. The system detects checkerboard patterns
                    and updates the camera parameters when the analysis completes.
                </p>

                <div class="d-flex flex-wrap gap-2 mb-3">
                    <button class="btn btn-primary" id="start-calibration-btn" disabled>
                        <i class="bi bi-gear me-1"></i>Start Calibration
                    </button>
                    <button class="btn btn-outline-secondary" id="stop-calibration-btn" disabled>
                        <i class="bi bi-x-circle me-1"></i>Stop
                    </button>
                    <button class="btn btn-outline-success" id="compute-calibration-btn" disabled>
                        <i class="bi bi-cpu me-1"></i>Compute Results
                    </button>
                </div>

                <div id="calibration-status" class="calibration-status" style="display: none;">
                    No videos to process
                </div>

                <div id="calibration-progress" style="display: none;">
                    <div class="progress mb-2">
                        <div class="progress-bar" role="progressbar" style="width: 0%"></div>
                    </div>
                    <div class="small text-muted" id="calibration-progress-text">Initializing...</div>
                </div>
            </div>
        `;

        // Render parameters panel
        this.renderParametersPanel(params);

        // Load camera previews
        this.loadCameraPreviews();

        // Update button states for initial render
        this.updateCaptureControls();
        this.renderCaptureSummary(this.calibrationStatus);
    },

    renderParametersPanel(params) {
        const parametersContainer = document.getElementById('calibration-parameters');

        const defaults = window.CameraApp.Config.CALIBRATION;
        const enableMultithreading = (params.enable_multithreading !== undefined)
            ? params.enable_multithreading
            : !!defaults.ENABLE_MULTITHREADING;
        const stereoCenterDiffVertical = (params.stereo_pose_max_center_diff_vertical !== undefined)
            ? params.stereo_pose_max_center_diff_vertical
            : (params.stereo_pose_max_center_diff !== undefined)
                ? params.stereo_pose_max_center_diff
                : (defaults.STEREO_CENTER_THRESHOLD_VERTICAL !== undefined
                    ? defaults.STEREO_CENTER_THRESHOLD_VERTICAL
                    : defaults.STEREO_CENTER_THRESHOLD);
        const stereoCenterDiffHorizontal = (params.stereo_pose_max_center_diff_horizontal !== undefined)
            ? params.stereo_pose_max_center_diff_horizontal
            : (defaults.STEREO_CENTER_THRESHOLD_HORIZONTAL !== undefined)
                ? defaults.STEREO_CENTER_THRESHOLD_HORIZONTAL
                : defaults.STEREO_CENTER_THRESHOLD;
        const stereoSpanRatio = (params.stereo_pose_max_normalized_span_ratio !== undefined)
            ? params.stereo_pose_max_normalized_span_ratio
            : (params.stereo_pose_max_scale_diff !== undefined)
                ? params.stereo_pose_max_scale_diff
                : (defaults.STEREO_NORMALIZED_SPAN_RATIO_THRESHOLD !== undefined)
                    ? defaults.STEREO_NORMALIZED_SPAN_RATIO_THRESHOLD
                    : defaults.STEREO_SCALE_THRESHOLD;
        const stereoTiltDiff = (params.stereo_pose_max_tilt_diff !== undefined)
            ? params.stereo_pose_max_tilt_diff
            : defaults.STEREO_TILT_THRESHOLD;

        parametersContainer.innerHTML = `
            <form id="calibration-params-form">
                <div class="mb-3">
                    <label class="form-label">Board Size</label>
                    <div class="row g-2">
                        <div class="col-6">
                            <input type="number" name="board_cols" class="form-control form-control-sm" 
                                   value="${params.board_cols || window.CameraApp.Config.CALIBRATION.DEFAULT_BOARD_COLS}" 
                                   min="3" max="20">
                            <small class="form-text">Columns</small>
                        </div>
                        <div class="col-6">
                            <input type="number" name="board_rows" class="form-control form-control-sm" 
                                   value="${params.board_rows || window.CameraApp.Config.CALIBRATION.DEFAULT_BOARD_ROWS}" 
                                   min="3" max="20">
                            <small class="form-text">Rows</small>
                        </div>
                    </div>
                </div>

                <div class="mb-3">
                    <label class="form-label">Square Size (mm)</label>
                    <input type="number" name="square_size" class="form-control form-control-sm" 
                           value="${params.square_size || window.CameraApp.Config.CALIBRATION.DEFAULT_SQUARE_SIZE}" 
                           min="1" max="100" step="0.1">
                </div>

                    <label class="form-label">Capture Duration (seconds)</label>
                    <input type="number" name="duration" class="form-control form-control-sm"
                           value="${params.duration || window.CameraApp.Config.CALIBRATION.DEFAULT_DURATION}"
                           min="10" max="300">
                </div>

                <div class="mb-3">
                    <label class="form-label">Frame Range</label>
                    <div class="row g-2">
                        <div class="col-6">
                            <input type="number" name="min_frames" class="form-control form-control-sm" 
                                   value="${params.min_frames || window.CameraApp.Config.CALIBRATION.MIN_FRAMES}" 
                                   min="5" max="50">
                            <small class="form-text">Min</small>
                        </div>
                        <div class="col-6">
                            <input type="number" name="max_frames" class="form-control form-control-sm" 
                                   value="${params.max_frames || window.CameraApp.Config.CALIBRATION.MAX_FRAMES}" 
                                   min="10" max="200">
                            <small class="form-text">Max</small>
                        </div>
                    </div>
                </div>

                <div class="mb-3">
                    <label class="form-label">Quality Threshold</label>
                    <input type="number" name="quality_threshold" class="form-control form-control-sm"
                           value="${params.quality_threshold || window.CameraApp.Config.CALIBRATION.QUALITY_THRESHOLD}"
                           min="1" max="100" step="0.1">
                    <small class="form-text">Minimum pattern quality score</small>
                </div>

                <div class="mb-3">
                    <label class="form-label">Time Tolerance (ms)</label>
                    <input type="number" name="time_tolerance_ms" class="form-control form-control-sm"
                           value="${(params.time_tolerance_ms !== undefined ? params.time_tolerance_ms : window.CameraApp.Config.CALIBRATION.TIME_TOLERANCE_MS)}"
                           min="0" max="200">
                    <small class="form-text">Maximum allowed timestamp difference between cameras</small>
                </div>


                <div class="mb-3">
                    <label class="form-label">Stereo Pose Filter</label>
                    <div class="row g-2 mb-2">
                        <div class="col-6">
                            <input type="number" name="stereo_pose_max_center_diff_vertical" class="form-control form-control-sm"
                                   value="${stereoCenterDiffVertical}" min="0" max="1" step="0.01">
                            <small class="form-text">Max vertical center diff (norm.)</small>
                        </div>
                        <div class="col-6">
                            <input type="number" name="stereo_pose_max_center_diff_horizontal" class="form-control form-control-sm"
                                   value="${stereoCenterDiffHorizontal}" min="0" max="1" step="0.01">
                            <small class="form-text">Max horizontal center diff (norm.)</small>
                        </div>
                    </div>
                    <div class="row g-2">
                        <div class="col-6">
                            <input type="number" name="stereo_pose_max_normalized_span_ratio" class="form-control form-control-sm"
                                   value="${stereoSpanRatio}" min="0" max="1" step="0.01">
                            <small class="form-text">Max normalized span diff</small>
                        </div>
                        <div class="col-6">
                            <input type="number" name="stereo_pose_max_tilt_diff" class="form-control form-control-sm"
                                   value="${stereoTiltDiff}" min="0" max="90" step="0.1">
                            <small class="form-text">Max tilt diff (°)</small>
                        </div>
                    </div>
                </div>

                <div class="form-check mb-3">
                    <input class="form-check-input" type="checkbox" name="delete_videos"
                           ${(params.delete_videos !== false) ? 'checked' : ''}>
                    <label class="form-check-label">Delete captures after calibration</label>
                </div>

                <div class="form-check mb-3">
                    <input class="form-check-input" type="checkbox" name="enable_multithreading"
                           ${enableMultithreading ? 'checked' : ''}>
                    <label class="form-check-label">Enable multithreaded image processing</label>
                    <small class="form-text text-muted">Process captured frames in parallel during calibration (uses more CPU)</small>
                </div>


                <div class="d-grid gap-2">
                    <button type="button" class="btn btn-outline-primary btn-sm" id="save-params-btn">
                        <i class="bi bi-check me-1"></i>Save Parameters
                    </button>
                    <button type="button" class="btn btn-outline-secondary btn-sm" id="reset-params-btn">
                        <i class="bi bi-arrow-counterclockwise me-1"></i>Reset to Defaults
                    </button>
                </div>
            </form>
        `;
    },

    async loadCameraPreviews() {
        try {
            const response = await window.CameraApp.API.getLiveCalibrationCameras();
            const cameras = Array.isArray(response?.cameras)
                ? response.cameras.filter(c => c.present !== false)
                : [];

            this.renderCameraPreviews(cameras);
        } catch (error) {
            console.error('Failed to load camera previews:', error);
        }
    },

    renderCameraPreviews(cameras) {
        const previewContainer = document.getElementById('calibration-preview');

        if (!previewContainer) {
            return;
        }

        const availableIds = new Set(cameras.map(camera => camera.id));
        this.selectedCameras = new Set(
            Array.from(this.selectedCameras).filter(id => availableIds.has(id))
        );


        const previewCard = `
            <div class="col-12">
                <div class="card border-0 shadow-sm">
                    <div class="card-body">
                        <h6 class="card-title mb-3">
                            <i class="bi bi-camera-reels me-2"></i>Camera Preview
                        </h6>
                        <div class="ratio ratio-16x9 bg-body-tertiary rounded position-relative overflow-hidden">
                            <img id="live-calibration-preview-img"
                                 class="w-100 h-100 object-fit-contain"
                                 alt="Live calibration preview"
                                 style="display: none;">
                            <div id="live-calibration-preview-placeholder"
                                 class="d-flex flex-column justify-content-center align-items-center text-muted h-100">
                                <i class="bi bi-camera-video-off fs-1 mb-2"></i>
                                <span>Start a capture to view live video</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        `;

        let camerasHtml = '';
        if (cameras.length === 0) {
            camerasHtml = `
                <div class="col-12">
                    <div class="text-center py-4 text-muted">
                        <i class="bi bi-camera-video-off display-4"></i>
                        <p class="mt-2">No cameras in calibration mode</p>
                        <small>Set cameras to calibration mode in the Settings tab</small>
                    </div>
                </div>
            `;
        } else {
            const selectedCameras = this.selectedCameras;
            camerasHtml = cameras.map(camera => `
                <div class="col-md-6 col-lg-4">
                    <div class="card">
                        <div class="card-body p-2">
                            <div class="d-flex justify-content-between align-items-center mb-2">
                                <h6 class="card-title mb-0">
                                    <i class="bi bi-camera me-1"></i>
                                    ${camera.id}
                                </h6>
                                <div class="form-check">
                                    <input class="form-check-input calibration-camera-select" type="checkbox"
                                           id="select-camera-${camera.id}"
                                           data-camera-id="${camera.id}"
                                           ${selectedCameras.has(camera.id) ? 'checked' : ''}>
                                    <label class="form-check-label small" for="select-camera-${camera.id}">
                                        Выбрать
                                    </label>
                                </div>
                            </div>

                            <div class="camera-feed">
                                <img src="${window.CameraApp.API.getPreviewMjpgUrl(camera.id)}"
                                     alt="Camera ${camera.id}"
                                     style="height: 100%; object-fit: contain;"
                                     onerror="this.style.display='none'; this.nextElementSibling.style.display='flex';">
                                <div class="camera-feed-overlay" style="display: none;">
                                    <div class="text-center">
                                        <i class="bi bi-exclamation-triangle text-warning"></i>
                                        <p class="mt-1 mb-0">Stream Error</p>
                                    </div>
                                </div>
                                <div class="camera-feed-label">CALIBRATION</div>
                            </div>
                        </div>
                    </div>
                </div>
            `).join('');
        }

        previewContainer.innerHTML = previewCard + camerasHtml;
        this.livePreviewElement = document.getElementById('live-calibration-preview-img');
        this.livePreviewPlaceholder = document.getElementById('live-calibration-preview-placeholder');

        this.setLivePreviewActive(!!this.calibrationStatus?.active);
        this.updateCaptureControls();
    },

    setLivePreviewActive(active) {
        const img = this.livePreviewElement;
        const placeholder = this.livePreviewPlaceholder;

        if (!img || !placeholder) {
            return;
        }

        if (active) {
            if (img.dataset.active !== 'true') {
                img.src = `${window.CameraApp.Config.API.CALIBRATION_NEW.VIDEO}?t=${Date.now()}`;
            }
            img.dataset.active = 'true';
            img.style.display = 'block';
            placeholder.style.display = 'none';
        } else {
            if (img.dataset.active === 'true') {
                img.removeAttribute('src');
            }
            img.dataset.active = 'false';
            img.style.display = 'none';
            placeholder.style.display = 'flex';
        }
    },
    getSelectedCameraIds() {
        return Array.from(this.selectedCameras);
    },

    hasCalibrationData() {
        return Boolean(this.calibrationStatus?.active);
    },


    isCaptureReady(status) {
        if (!status || !status.active) {
            return false;
        }

        if (!['mono', 'stereo'].includes(status.mode)) {
            return false;
        }

        const progressCurrent = Number(status?.progress?.current);
        const current = Number.isFinite(progressCurrent) ? Math.max(progressCurrent, 0) : 0;
        const progressMax = Number(status?.progress?.max);
        const max = Number.isFinite(progressMax) ? Math.max(progressMax, 0) : 0;
        const minFramesValue = Number(status?.config?.min_frames);
        const minFrames = Number.isFinite(minFramesValue) ? Math.max(minFramesValue, 0) : 0;
        const required = max > 0 ? max : minFrames;

        if (required <= 0) {
            return false;
        }

        return current >= required;
    },

    updateCaptureControls() {
        const monoBtn = document.getElementById('start-mono-capture-btn');
        const stereoBtn = document.getElementById('start-stereo-capture-btn');
        const calibBtn = document.getElementById('start-calibration-btn');
        const checkboxElements = document.querySelectorAll('.calibration-camera-select');
        const computeBtn = document.getElementById('compute-calibration-btn');

        const selected = this.getSelectedCameraIds();
        const status = this.calibrationStatus || {};
        const active = !!status.active;
        const calibrating = !!status.calibrating;
        const computing = !!status.compute_in_progress;
        const busy = calibrating || computing;
        const captureReady = this.isCaptureReady(status);

        if (monoBtn) {
            monoBtn.disabled = active || busy || selected.length !== 1;
        }

        if (stereoBtn) {
            stereoBtn.disabled = active || busy || selected.length !== 2;
        }

        checkboxElements.forEach(checkbox => {
            checkbox.disabled = active || busy;
        });

        if (calibBtn) {
            calibBtn.disabled = !active || calibrating || computing;
        }

        if (computeBtn) {
            computeBtn.disabled = !active || calibrating || computing || !captureReady;
        }
    },

    updateCaptureStatus(message = '', variant = 'info') {
        const statusEl = document.getElementById('capture-status');
        if (!statusEl) {
            return;
        }

        if (!message) {
            statusEl.style.display = 'none';
            statusEl.className = 'calibration-status';
            statusEl.textContent = '';
            return;
        }

        const variantClass = {
            success: 'success',
            error: 'error',
            danger: 'error',
            warning: 'warning',
            info: 'info'
        }[variant] || 'info';

        statusEl.style.display = 'block';
        statusEl.className = `calibration-status ${variantClass}`;
        statusEl.innerHTML = message;
    },

    showMonoProgressPanel(cameraId = '') {
        const panel = document.getElementById('mono-progress-panel');
        if (!panel) {
            return;
        }
        panel.style.display = 'block';

        const bar = document.getElementById('mono-progress-bar');
        if (bar) {
            bar.style.width = '0%';
            bar.textContent = '0%';
            bar.setAttribute('aria-valuenow', '0');
        }

        const countEl = document.getElementById('mono-progress-count');
        if (countEl) {
            countEl.textContent = '0 / 0 frames';
        }

        const hintEl = document.getElementById('mono-progress-hint');
        if (hintEl) {
            hintEl.textContent = cameraId ? `Preparing ${cameraId}...` : 'Preparing...';
        }

        const badgeEl = document.getElementById('mono-progress-visibility');
        if (badgeEl) {
            badgeEl.style.display = 'inline-block';
            badgeEl.className = 'badge bg-secondary';
            badgeEl.textContent = 'Initializing';
        }

        const previewWrapper = document.getElementById('mono-progress-preview-wrapper');
        if (previewWrapper) {
            previewWrapper.style.display = 'none';
        }

        const previewImg = document.getElementById('mono-progress-preview');
        if (previewImg) {
            previewImg.removeAttribute('src');
        }

        const summaryEl = document.getElementById('mono-progress-summary');
        if (summaryEl) {
            summaryEl.style.display = 'none';
            summaryEl.innerHTML = '';
        }
    },

    hideMonoProgressPanel() {
        const panel = document.getElementById('mono-progress-panel');
        if (panel) {
            panel.style.display = 'none';
        }
    },

    updateMonoProgressUI(status = {}) {
        const progress = Number.isFinite(status.progress) ? Math.min(Math.max(status.progress, 0), 100) : 0;
        const framesCollected = Number.isFinite(status.frames_collected) ? Math.max(status.frames_collected, 0) : 0;
        const framesNeeded = Number.isFinite(status.frames_needed) && status.frames_needed > 0
            ? status.frames_needed
            : '?';

        const bar = document.getElementById('mono-progress-bar');
        if (bar) {
            bar.style.width = `${progress}%`;
            bar.textContent = `${progress}%`;
            bar.setAttribute('aria-valuenow', String(progress));
        }

        const countEl = document.getElementById('mono-progress-count');
        if (countEl) {
            countEl.textContent = `${framesCollected} / ${framesNeeded} frames`;
        }

        const hintEl = document.getElementById('mono-progress-hint');
        if (hintEl) {
            hintEl.textContent = status.hint || 'Searching for checkerboard...';
        }

        const badgeEl = document.getElementById('mono-progress-visibility');
        if (badgeEl) {
            if (status.status === 'done') {
                badgeEl.style.display = 'inline-block';
                badgeEl.className = 'badge bg-primary';
                badgeEl.textContent = 'Completed';
            } else if (status.status === 'error') {
                badgeEl.style.display = 'inline-block';
                badgeEl.className = 'badge bg-danger';
                badgeEl.textContent = 'Error';
            } else if (status.status === 'processing') {
                badgeEl.style.display = 'inline-block';
                badgeEl.className = 'badge bg-info text-dark';
                badgeEl.textContent = 'Processing';
            } else if (status.board_visible) {
                badgeEl.style.display = 'inline-block';
                badgeEl.className = 'badge bg-success';
                badgeEl.textContent = 'Board detected';
            } else {
                badgeEl.style.display = 'inline-block';
                badgeEl.className = 'badge bg-warning text-dark';
                badgeEl.textContent = 'Searching...';
            }
        }

        const previewWrapper = document.getElementById('mono-progress-preview-wrapper');
        const previewImg = document.getElementById('mono-progress-preview');
        if (previewWrapper && previewImg) {
            if (status.preview) {
                previewWrapper.style.display = 'block';
                if (previewImg.src !== status.preview) {
                    previewImg.src = status.preview;
                }
            } else if (!previewImg.src) {
                previewWrapper.style.display = 'none';
            }
        }

        const summaryEl = document.getElementById('mono-progress-summary');
        if (summaryEl) {
            if (status.result) {
                const frames = Number.isFinite(status.result.frames) ? status.result.frames : framesCollected;
                const rmsValue = Number.isFinite(status.result.rms) ? status.result.rms : null;
                const parts = [`Frames used: <strong>${frames}</strong>`];
                if (rmsValue !== null) {
                    parts.push(`RMS error: <strong>${rmsValue.toFixed(3)}</strong>`);
                }
                if (status.result.file) {
                    parts.push(`Saved to <code>${status.result.file}</code>`);
                }
                summaryEl.innerHTML = parts.join(' • ');
                summaryEl.style.display = 'block';
            } else if (status.status === 'error' && status.error) {
                summaryEl.innerHTML = `<span class="text-danger">${status.error}</span>`;
                summaryEl.style.display = 'block';
            } else {
                summaryEl.style.display = 'none';
                summaryEl.innerHTML = '';
            }
        }
    },

    stopMonoCapturePolling() {
        if (this.monoCaptureTimer) {
            clearInterval(this.monoCaptureTimer);
            this.monoCaptureTimer = null;
        }
    },

    async pollMonoCaptureProgress() {
        if (!this.activeMonoJobId) {
            return false;
        }
        try {
            const status = await window.CameraApp.API.getMonoCaptureProgress(this.activeMonoJobId);
            if (!status) {
                return true;
            }

            this.updateMonoProgressUI(status);

            if (status.status === 'done') {
                await this.handleMonoCaptureSuccess(status);
                return false;
            }

            if (status.status === 'error') {
                const message = status.error || 'Mono calibration failed';
                await this.handleMonoCaptureError(new Error(message));
                return false;
            }

            const cameraLabel = status.camera || this.activeMonoCameraId || '';
            if (status.status === 'processing') {
                let message = `Processing captured frames for <strong>${cameraLabel}</strong>.`;
                if (status.hint) {
                    message += `<div class="small mt-1">${status.hint}</div>`;
                }
                this.updateCaptureStatus(message, 'info');
                return true;
            }

            const framesCollected = Number.isFinite(status.frames_collected) ? status.frames_collected : 0;
            const framesNeeded = Number.isFinite(status.frames_needed) && status.frames_needed > 0
                ? status.frames_needed
                : '?';
            let message = `Collecting frames for <strong>${cameraLabel}</strong> (${framesCollected}/${framesNeeded})`;
            if (status.hint) {
                message += `<div class="small mt-1">${status.hint}</div>`;
            }
            this.updateCaptureStatus(message, status.board_visible ? 'info' : 'warning');

            return true;
        } catch (error) {
            await this.handleMonoCaptureError(error);
            return false;
        }
    },

    async handleMonoCaptureSuccess(status) {
        const cameraId = status?.result?.camera || status?.camera || this.activeMonoCameraId || '';
        this.updateCaptureStatus(`Mono capture complete for <strong>${cameraId}</strong>.`, 'success');
        window.CameraApp.UI.showToast(`${window.CameraApp.Config.SUCCESS.CAPTURE_COMPLETED} (${cameraId})`, 'success');
        this.updateMonoProgressUI(status);
        await this.finalizeMonoCapture(true, { status });
    },

    async handleMonoCaptureError(error) {
        const cameraId = this.activeMonoCameraId;
        const cameraText = cameraId ? ` for <strong>${cameraId}</strong>` : '';
        const message = error?.message || String(error || 'Unknown error');
        this.updateCaptureStatus(`Mono capture failed${cameraText}: ${message}`, 'error');
        window.CameraApp.UI.showToast(`Mono capture failed: ${message}`, 'danger');
        await this.finalizeMonoCapture(false, { error });
    },

    async finalizeMonoCapture(success, { status = null } = {}) {
        this.stopMonoCapturePolling();
        const monoBtn = document.getElementById('start-mono-capture-btn');
        if (monoBtn) {
            window.CameraApp.UI.setButtonLoading(monoBtn, false);
        }

        this.activeMonoJobId = null;
        this.activeMonoCameraId = null;

        try {
            await this.releaseCalibrationSession();
        } catch (sessionError) {
            console.warn('Failed to release calibration session:', sessionError);
        }

        this.captureInProgress = false;
        this.updateCaptureControls();

        if (!success) {
            this.hideMonoProgressPanel();
        }

        await this.refreshCalibrationStatus();
    },



    renderCaptureSummary(status, error = null) {
        const summaryEl = document.getElementById('capture-summary');
        if (!summaryEl) {
            return;
        }

        if (error) {
            summaryEl.innerHTML = `<span class="text-danger">${error}</span>`;
            return;
        }

        if (!status || !status.active) {
            summaryEl.textContent = 'Calibration idle. Select cameras and start a capture to begin.';
            return;
        }

        const parts = [];
        if (status.mode) {
            parts.push(`Mode: ${status.mode}`);
        }
        if (status.camera_a) {
            parts.push(`Primary: ${status.camera_a}`);
        }
        if (status.camera_b) {
            parts.push(`Secondary: ${status.camera_b}`);
        }
        const current = Number(status.progress?.current) || 0;
        const max = Number(status.progress?.max) || 0;
        parts.push(`Frames: ${max > 0 ? `${current}/${max}` : current}`);

        if (status.compute_in_progress) {
            parts.push('Computing');
        } else if (status.calibrating) {
            parts.push('Calibrating');
        } else if (status.running) {
            parts.push('Capturing');
        } else {
            parts.push('Ready');
        }

        const monoCount = Array.isArray(status.mono_results) ? status.mono_results.length : 0;
        const stereoCount = Array.isArray(status.stereo_results) ? status.stereo_results.length : 0;
        if (monoCount + stereoCount > 0) {
            parts.push(`Results: ${monoCount + stereoCount}`);
        }

        summaryEl.innerHTML = parts.map(part => `<span>${part}</span>`).join(' • ');
    },
    async refreshCalibrationStatus() {
        try {
            const status = await window.CameraApp.API.getLiveCalibrationStatus();
            this.calibrationStatus = status;
            this.captureInProgress = !!status?.running;
            window.CameraApp.State.isCalibrating = !!(status?.calibrating || status?.compute_in_progress);

            if (status?.active) {
                this.setLivePreviewActive(true);
                this.ensureStatusTimer();
            } else {
                this.setLivePreviewActive(false);
                this.stopStatusTimer();
                this.updateCaptureStatus('', 'info');
            }

            this.renderCaptureSummary(status);
            this.updateCalibrationProgress(status);
            this.updateCaptureControls();
            this.maybeTriggerAutoCompute(status);
            return status;
        } catch (error) {
            console.warn('Failed to refresh calibration status:', error);
            this.calibrationStatus = null;
            this.captureInProgress = false;
            window.CameraApp.State.isCalibrating = false;

            this.renderCaptureSummary(null, 'Unable to load capture status');
            this.updateCalibrationProgress(null);
            this.setLivePreviewActive(false);
            this.stopStatusTimer();
            this.updateCaptureStatus('', 'info');
            this.updateCaptureControls();
            return null;
        }

    },

    getBoardConfiguration() {
        const defaults = window.CameraApp.Config.CALIBRATION;
        let boardCols = null;
        let boardRows = null;

        const form = document.getElementById('calibration-params-form');
        if (form) {
            const formValues = window.CameraApp.UI.getFormData(form);
            const colsValue = Number(formValues.board_cols);
            const rowsValue = Number(formValues.board_rows);
            boardCols = Number.isFinite(colsValue) && colsValue > 0 ? colsValue : null;
            boardRows = Number.isFinite(rowsValue) && rowsValue > 0 ? rowsValue : null;
        }

        const params = this.calibrationParams || defaults;
        if (boardCols === null) {
            const storedCols = Number(params.board_cols);
            boardCols = Number.isFinite(storedCols) && storedCols > 0 ? storedCols : defaults.DEFAULT_BOARD_COLS;
        }

        if (boardRows === null) {
            const storedRows = Number(params.board_rows);
            boardRows = Number.isFinite(storedRows) && storedRows > 0 ? storedRows : defaults.DEFAULT_BOARD_ROWS;
        }

        return { boardCols, boardRows };
    },

    getCalibrationConfigPayload() {
        const defaults = window.CameraApp.Config.CALIBRATION;
        const form = document.getElementById('calibration-params-form');
        const formValues = form ? window.CameraApp.UI.getFormData(form) : {};
        const { boardCols, boardRows } = this.getBoardConfiguration();

        const readNumber = (value, fallback) => {
            const parsed = Number(value);
            return Number.isFinite(parsed) ? parsed : fallback;
        };

        const verticalFallback = defaults.STEREO_CENTER_THRESHOLD_VERTICAL ?? defaults.STEREO_CENTER_THRESHOLD;
        const horizontalFallback = defaults.STEREO_CENTER_THRESHOLD_HORIZONTAL ?? defaults.STEREO_CENTER_THRESHOLD;

        const payload = {
            pattern_cols: boardCols,
            pattern_rows: boardRows,
            square_size: readNumber(formValues.square_size, defaults.DEFAULT_SQUARE_SIZE),
            min_frames: readNumber(formValues.min_frames, defaults.MIN_FRAMES),
            max_frames: readNumber(formValues.max_frames, defaults.MAX_FRAMES),
            max_center_diff_vertical: readNumber(formValues.stereo_pose_max_center_diff_vertical, verticalFallback),
            max_center_diff_horizontal: readNumber(formValues.stereo_pose_max_center_diff_horizontal, horizontalFallback),
            max_tilt_diff: readNumber(formValues.stereo_pose_max_tilt_diff, defaults.STEREO_TILT_THRESHOLD)
        };

        return payload;
    },
    async ensureCalibrationSession() {
        // No-op with the new live calibration API; kept for backward compatibility.
        return Promise.resolve();
    },

    async releaseCalibrationSession(force = false) {
        if (force) {
            await this.stopAutoCalibration({ silent: true });
        }
    },

    async startMonoCapture() {
        const monoBtn = document.getElementById('start-mono-capture-btn');
        const selected = this.getSelectedCameraIds();

        if (selected.length !== 1) {
            window.CameraApp.UI.showToast('Select exactly one camera for mono capture', 'warning');
            return;
        }

        const cameraId = selected[0];
        const frameInput = document.getElementById('mono-frame-count');
        const framesValue = frameInput?.value?.trim();

        const frames = framesValue ? Number(framesValue) : null;

        if (framesValue && (!Number.isFinite(frames) || frames <= 0)) {
            window.CameraApp.UI.showToast('Mono frame count must be a positive number', 'warning');
            return;
        }

        const config = this.getCalibrationConfigPayload();
        if (Number.isFinite(frames) && frames > 0) {
            config.max_frames = frames;
        }

        try {
            if (monoBtn) {
                window.CameraApp.UI.setButtonLoading(monoBtn, true, 'Starting...');
            }

            this.updateCaptureStatus(`Starting mono capture for <strong>${cameraId}</strong>...`, 'info');
            this.autoComputeEnabled = false;
            this.autoComputeTriggered = false;

            const response = await window.CameraApp.API.startLiveCalibration({
                mode: 'mono',
                camera_a: cameraId,
                config
            });

            if (response?.status !== 'ok') {
                throw new Error(response?.error || 'Failed to start mono capture');
            }

            this.setLivePreviewActive(true);
            this.ensureStatusTimer();
            await this.refreshCalibrationStatus();

            window.CameraApp.UI.showToast(`Mono capture started for ${cameraId}`, 'success');
        } catch (error) {
            const message = error?.message || String(error || 'Unknown error');
            this.updateCaptureStatus(`Mono capture failed: ${message}`, 'error');
            window.CameraApp.UI.showToast(`Mono capture failed: ${message}`, 'danger');
        } finally {
            if (monoBtn) {
                window.CameraApp.UI.setButtonLoading(monoBtn, false);
            }
        }
    },

    async startStereoCapture() {
        const stereoBtn = document.getElementById('start-stereo-capture-btn');
        const selected = this.getSelectedCameraIds();

        if (selected.length !== 2) {
            window.CameraApp.UI.showToast('Select exactly two cameras for stereo capture', 'warning');
            return;
        }
        const frameInput = document.getElementById('stereo-frame-count');

        const framesValue = frameInput?.value?.trim();
        const frames = framesValue ? Number(framesValue) : null;

        if (framesValue && (!Number.isFinite(frames) || frames <= 0)) {
            window.CameraApp.UI.showToast('Stereo frame count must be a positive number', 'warning');
            return;
        }

        const sortedCameras = [...selected].sort();

        const config = this.getCalibrationConfigPayload();
        if (Number.isFinite(frames) && frames > 0) {
            config.max_frames = frames;
        }
        try {
            if (stereoBtn) {
                window.CameraApp.UI.setButtonLoading(stereoBtn, true, 'Starting...');
            }

            this.updateCaptureStatus(`Starting stereo capture for <strong>${sortedCameras.join(' & ')}</strong>...`, 'info');
            this.autoComputeEnabled = false;
            this.autoComputeTriggered = false;
            const response = await window.CameraApp.API.startLiveCalibration({
                mode: 'stereo',
                camera_a: sortedCameras[0],
                camera_b: sortedCameras[1],
                config
            });

            if (response?.status !== 'ok') {
                throw new Error(response?.error || 'Failed to start stereo capture');
            }

            this.setLivePreviewActive(true);
            this.ensureStatusTimer();
            await this.refreshCalibrationStatus();

            window.CameraApp.UI.showToast(`Stereo capture started for ${sortedCameras.join(' & ')}`, 'success');
        } catch (error) {
            const message = error?.message || String(error || 'Unknown error');
            this.updateCaptureStatus(`Stereo capture failed: ${message}`, 'error');
            window.CameraApp.UI.showToast(`Stereo capture failed: ${message}`, 'danger');
        } finally {
            if (stereoBtn) {
                window.CameraApp.UI.setButtonLoading(stereoBtn, false);
            }
        }

    },

    async startAutoCalibration() {
        const startBtn = document.getElementById('start-calibration-btn');
        const stopBtn = document.getElementById('stop-calibration-btn');
        const computeBtn = document.getElementById('compute-calibration-btn');

        if (!this.calibrationStatus?.active) {
            window.CameraApp.UI.showToast('Start a mono or stereo capture before calibrating', 'warning');
            return;
        }

        try {
            if (startBtn) {
                window.CameraApp.UI.setButtonLoading(startBtn, true, 'Starting...');
            }
            this.updateCaptureStatus('Starting calibration...', 'info');
            this.autoComputeEnabled = true;
            this.autoComputeTriggered = false;

            const response = await window.CameraApp.API.calibrateLiveCalibration();

            if (response?.status !== 'ok') {
                throw new Error(response?.error || 'Failed to start calibration');
            }
            window.CameraApp.State.isCalibrating = true;
            if (stopBtn) stopBtn.disabled = false;
            if (computeBtn) computeBtn.disabled = true;

            this.ensureStatusTimer();
            await this.refreshCalibrationStatus();
            window.CameraApp.UI.showToast('Calibration started', 'success');
        } catch (error) {
            const message = error?.message || String(error || 'Unknown error');
            this.autoComputeEnabled = false;
            this.autoComputeTriggered = false;
            this.updateCaptureStatus(`Calibration failed: ${message}`, 'error');
            window.CameraApp.UI.showToast(`Calibration failed: ${message}`, 'danger');
        } finally {
            if (startBtn) {
                window.CameraApp.UI.setButtonLoading(startBtn, false);
            }
            this.updateCaptureControls();
        }
    },

    async computeCalibrationResults(options = {}) {
        const opts = typeof options === 'boolean' ? { auto: options } : options;
        const { auto = false } = opts;
        const computeBtn = document.getElementById('compute-calibration-btn');

        if (!this.calibrationStatus?.active) {
            window.CameraApp.UI.showToast('Nothing to compute yet — start a capture first', 'warning');
            return;
        }

        try {
            if (!auto && computeBtn) {
                window.CameraApp.UI.setButtonLoading(computeBtn, true, 'Computing...');
            }
            this.updateCaptureStatus('Computing calibration results...', 'info');
            this.autoComputeEnabled = false;

            const response = await window.CameraApp.API.computeLiveCalibration();
            if (response?.status !== 'ok') {
                throw new Error(response?.error || 'Calibration compute failed');
            }

            this.showCalibrationResults(response);
            this.updateCaptureStatus('Calibration results updated.', 'success');
            window.CameraApp.UI.showToast('Calibration results updated', 'success');
        } catch (error) {
            const message = error?.message || String(error || 'Unknown error');
            this.updateCaptureStatus(`Compute failed: ${message}`, 'error');
            window.CameraApp.UI.showToast(`Compute failed: ${message}`, 'danger');
            if (auto) {
                this.autoComputeTriggered = false;
            }
        } finally {
            if (!auto && computeBtn) {
                window.CameraApp.UI.setButtonLoading(computeBtn, false);
            }
            await this.refreshCalibrationStatus();
        }
    },

updateCalibrationProgress(status) {
    const statusEl = document.getElementById('calibration-status');
    const progressEl = document.getElementById('calibration-progress');
    const progressBar = document.querySelector('#calibration-progress .progress-bar');
    const progressText = document.getElementById('calibration-progress-text');

    if (!statusEl || !progressEl || !progressBar || !progressText) {
        return;
    }

    if (!status || !status.active) {
        statusEl.style.display = 'block';
        statusEl.className = 'calibration-status';
        statusEl.innerHTML = 'Calibration idle.';
        progressEl.style.display = 'none';
        return;
    }

    progressEl.style.display = 'block';

    // НE используем статус-бар для подсчета кадров - только для финального результата
    const progressCurrent = Number(status?.progress?.current);
    const current = Number.isFinite(progressCurrent) ? Math.max(progressCurrent, 0) : 0;
    const progressMax = Number(status?.progress?.max);
    const max = Number.isFinite(progressMax) ? Math.max(progressMax, 0) : 0;
    
    // Показываем прогресс только если калибровка завершена или идет вычисление
    if (status.compute_in_progress || (status.mono_results && status.mono_results.length > 0) || 
        (status.stereo_results && status.stereo_results.length > 0)) {
        const percent = max > 0 ? Math.min(100, (current / max) * 100) : 100;
        progressBar.style.width = `${percent}%`;
        progressBar.setAttribute('aria-valuenow', String(current));
        progressBar.setAttribute('aria-valuemax', String(max || 100));
        progressBar.textContent = max > 0 ? `${current} / ${max}` : `${current}`;
    } else {
        // Во время захвата показываем только индикатор активности
        progressBar.style.width = '0%';
        progressBar.setAttribute('aria-valuenow', '0');
        progressBar.setAttribute('aria-valuemax', String(max || 100));
        progressBar.textContent = 'Capturing...';
    }

        let headline = '';
        if (status.error) {
            statusEl.className = 'calibration-status error';
            headline = `Error: ${status.error}`;
        } else if (status.compute_in_progress) {
            statusEl.className = 'calibration-status info';
            headline = 'Computing calibration results...';
        } else if (status.calibrating) {
            statusEl.className = 'calibration-status info';
            headline = 'Calibration in progress';
        } else if (status.running) {
            statusEl.className = status.pattern_visible ? 'calibration-status success' : 'calibration-status warning';
            headline = 'Capturing frames';
        } else {
            statusEl.className = 'calibration-status success';
            headline = 'Calibration ready';
        }

        statusEl.style.display = 'block';
        statusEl.innerHTML = headline;

        const hint = status.hint
            || (status.pattern_visible ? 'Checkerboard detected' : 'Move the pattern into view');
        progressText.textContent = hint;

        this.updateCaptureStatus(hint, status.pattern_visible ? 'success' : 'warning');
    },


    maybeTriggerAutoCompute(status) {
        if (!this.autoComputeEnabled || this.autoComputeTriggered) {
            return;
        }

        if (!status || !status.active || status.compute_in_progress) {
            return;
        }

        if (!['mono', 'stereo'].includes(status.mode)) {
            return;
        }

        if (status.calibrating || !this.isCaptureReady(status)) {
            return;
        }

        const hasResults = (Array.isArray(status.mono_results) && status.mono_results.length > 0)
            || (Array.isArray(status.stereo_results) && status.stereo_results.length > 0);
        if (hasResults) {
            this.autoComputeEnabled = false;
            this.autoComputeTriggered = true;
            return;
        }

        this.autoComputeTriggered = true;
        this.computeCalibrationResults({ auto: true }).catch(error => {
            console.warn('Auto calibration compute failed:', error);
        });
    },

    showCalibrationResults(results) {
        const monoResults = Array.isArray(results?.mono_results) ? results.mono_results : [];
        const stereoResults = Array.isArray(results?.stereo_results) ? results.stereo_results : [];

        if (monoResults.length === 0 && stereoResults.length === 0) {
            return;
        }

        let resultsHtml = '<div class="mt-3"><strong>Results:</strong><br>';
        monoResults.forEach(result => {
            const errorValue = Number(result.reprojection_error);
            const errorText = Number.isFinite(errorValue) ? errorValue.toFixed(2) : 'n/a';
            resultsHtml += `
                <div class="d-flex flex-wrap align-items-center mb-1">
                    <i class="bi bi-check-circle text-success me-2"></i>
                    <span>${result.camera_id}</span>
                    <small class="text-muted ms-2">${result.frames_used ?? '?'} frames, error: ${errorText}</small>
                </div>
            `;
        });

        if (stereoResults.length > 0) {
            resultsHtml += '<br><strong>Stereo Pairs:</strong><br>';
            stereoResults.forEach(result => {
                const pairLabel = result.camera_pair
                    || `${result.camera_a || '?'} & ${result.camera_b || '?'}`;
                const errorValue = Number(result.reprojection_error);
                const errorText = Number.isFinite(errorValue) ? errorValue.toFixed(2) : 'n/a';
                resultsHtml += `
                    <div class="d-flex flex-wrap align-items-center mb-1">
                        <i class="bi bi-check-circle text-success me-2"></i>
                        <span>${pairLabel}</span>
                        <small class="text-muted ms-2">${result.frames_used ?? '?'} frames, error: ${errorText}</small>
                    </div>
                `;
            });
        }

        resultsHtml += '</div>';

        const statusEl = document.getElementById('calibration-status');
        statusEl.innerHTML += resultsHtml;
    },

    async stopAutoCalibration(options = {}) {
        const opts = typeof options === 'boolean' ? { silent: options } : options;
        const { silent = false } = opts;
        try {
            const response = await window.CameraApp.API.stopLiveCalibration();
            if (response?.status !== 'ok') {
                throw new Error(response?.error || 'Failed to stop calibration');
            }
        } catch (error) {
            if (!silent) {
                window.CameraApp.UI.showToast(`Failed to stop calibration: ${error?.message || error}`, 'danger');
            }
        } finally {
            this.stopStatusTimer();
            this.setLivePreviewActive(false);
            this.autoComputeEnabled = false;
            this.autoComputeTriggered = false;

            window.CameraApp.State.isCalibrating = false;
            this.updateCaptureStatus('', 'info');
            await this.refreshCalibrationStatus();
            this.updateCaptureControls();
            if (!silent) {
                window.CameraApp.UI.showToast('Calibration stopped', 'info');
            }
        }
    },

    async saveCalibrationParams() {
        const form = document.getElementById('calibration-params-form');
        const saveBtn = document.getElementById('save-params-btn');
        
        if (!form) return; // Form might not be rendered yet
        
        if (!window.CameraApp.UI.validateForm(form)) {
            window.CameraApp.UI.showToast('Please check all parameters', 'warning');
            return;
        }

        try {
            if (saveBtn) {
                window.CameraApp.UI.setButtonLoading(saveBtn, true, 'Saving...');
            }
            
            const formData = window.CameraApp.UI.getFormData(form);
            await window.CameraApp.API.setCalibrationParams(formData);
            this.calibrationParams = { ...this.calibrationParams, ...formData };

            window.CameraApp.UI.showToast('Parameters saved', 'success');
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to save parameters: ${error.message}`, 'danger');
        } finally {
            if (saveBtn) {
                window.CameraApp.UI.setButtonLoading(saveBtn, false);
            }
        }
    },

    async resetCalibrationParams() {
        const confirmed = await window.CameraApp.UI.confirm(
            'Reset all calibration parameters to default values?',
            'Reset Parameters'
        );

        if (!confirmed) return;

        try {
            const defaultParams = window.CameraApp.Config.CALIBRATION;
            await window.CameraApp.API.setCalibrationParams(defaultParams);

            this.calibrationParams = defaultParams;
            this.renderParametersPanel(defaultParams);
            window.CameraApp.UI.showToast('Parameters reset to defaults', 'success');
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to reset parameters: ${error.message}`, 'danger');
        }
    },

    startPreviewTimer() {
        // Refresh camera previews every 30 seconds
        this.previewTimer = setInterval(() => {
            this.loadCameraPreviews();
        }, 30000);
    },

    stopPreviewTimer() {
        if (this.previewTimer) {
            clearInterval(this.previewTimer);
            this.previewTimer = null;
        }
    },

    stopStatusTimer() {
        if (this.statusTimer) {
            clearInterval(this.statusTimer);
            this.statusTimer = null;
        }
    },

    ensureStatusTimer() {
        if (this.statusTimer) {
            return;
        }

        this.statusTimer = setInterval(() => {
            this.refreshCalibrationStatus().catch(error => {
                console.warn('Calibration status polling failed:', error);
            });
        }, 1000);
    },

    cleanup() {
        this.stopPreviewTimer();
        this.stopStatusTimer();
        this.setLivePreviewActive(false);
        this.stopAutoCalibration({ silent: true }).catch(error => {
            console.warn('Failed to stop calibration during cleanup:', error);
        });

        if (this.paramsSaveTimeout) {
            clearTimeout(this.paramsSaveTimeout);
            this.paramsSaveTimeout = null;
        }
    },

    // Tab activation/deactivation
    async activate() {
        await window.CameraApp.API.enablePreview(false);
        window.CameraApp.State.previewEnabled = false;

        await this.loadCalibrationParams();
        await this.refreshCalibrationStatus();
        this.startPreviewTimer();
    },

    deactivate() {
        this.cleanup();

    }
};