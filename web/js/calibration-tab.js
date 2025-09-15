// Calibration Tab Module
window.CameraApp = window.CameraApp || {};

window.CameraApp.CalibrationTab = {
    previewTimer: null,
    statusTimer: null,
    recordingTimer: null,
    paramsSaveTimeout: null,
    currentCalibration: null,
    
    async init() {
        this.setupEventListeners();
        await this.loadCalibrationParams();
    },

    setupEventListeners() {
        // Event delegation for dynamic elements
        document.addEventListener('click', this.handleDynamicEvents.bind(this));
        document.addEventListener('change', this.handleDynamicChanges.bind(this));
    },

    handleDynamicEvents(event) {
        const target = event.target;

        // Recording controls
        if (target.id === 'start-recording-btn') {
            this.startRecording();
        }
        if (target.id === 'stop-recording-btn') {
            this.stopRecording();
        }

        // Calibration controls
        if (target.id === 'start-calibration-btn') {
            this.startAutoCalibration();
        }
        if (target.id === 'stop-calibration-btn') {
            this.stopAutoCalibration();
        }

        // Parameter controls
        if (target.id === 'save-params-btn') {
            this.saveCalibrationParams();
        }
        if (target.id === 'reset-params-btn') {
            this.resetCalibrationParams();
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
    },

    async loadCalibrationParams() {
        try {
            const params = await window.CameraApp.API.getCalibrationParams();
            this.renderCalibrationInterface(params);
        } catch (error) {
            console.error('Failed to load calibration parameters:', error);
            this.renderCalibrationInterface(window.CameraApp.Config.CALIBRATION);
        }
    },

    renderCalibrationInterface(params) {
        // Render main calibration content
        const content = document.getElementById('calibration-content');
        content.innerHTML = `
            <div class="calibration-step">
                <h6 class="fw-bold mb-3">
                    <i class="bi bi-1-circle me-2"></i>Record Calibration Videos
                </h6>
                <p class="text-muted mb-3">
                    Record videos of calibration patterns from all cameras simultaneously.
                    Make sure the checkerboard pattern is clearly visible and covers different areas of the frame.
                </p>
                
                <div class="d-flex gap-2 mb-3">
                    <button class="btn btn-success" id="start-recording-btn">
                        <i class="bi bi-record-circle me-1"></i>Start Recording
                    </button>
                    <button class="btn btn-danger" id="stop-recording-btn" disabled>
                        <i class="bi bi-stop-circle me-1"></i>Stop Recording
                    </button>
                </div>
                
                <div id="recording-status" class="calibration-status" style="display: none;">
                    Ready to record
                </div>
            </div>

            <div class="calibration-step">
                <h6 class="fw-bold mb-3">
                    <i class="bi bi-2-circle me-2"></i>Process & Calibrate
                </h6>
                <p class="text-muted mb-3">
                    Automatically process the recorded videos and perform calibration.
                    This will detect checkerboard patterns and calculate camera parameters.
                </p>


                <div class="row g-2 mb-3">
                    <div class="col-6">
                        <input type="text" id="camera-a-id" class="form-control form-control-sm" placeholder="Camera A ID">
                    </div>
                    <div class="col-6">
                        <input type="text" id="camera-b-id" class="form-control form-control-sm" placeholder="Camera B ID">
                    </div>
                </div>

                <div class="d-flex gap-2 mb-3">
                    <button class="btn btn-primary" id="start-calibration-btn" disabled>
                        <i class="bi bi-gear me-1"></i>Start Calibration
                    </button>
                    <button class="btn btn-outline-secondary" id="stop-calibration-btn" disabled>
                        <i class="bi bi-x-circle me-1"></i>Stop
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
    },

    renderParametersPanel(params) {
        const parametersContainer = document.getElementById('calibration-parameters');
        
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

                <div class="mb-3">
                    <label class="form-label">Recording Duration (seconds)</label>
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

                <div class="form-check mb-3">
                    <input class="form-check-input" type="checkbox" name="delete_videos" 
                           ${(params.delete_videos !== false) ? 'checked' : ''}>
                    <label class="form-check-label">Delete videos after calibration</label>
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
            const cameras = await window.CameraApp.API.getConfiguredCameras();
            const calibCameras = cameras.filter(c => c.mode === 'calibration' && c.present);
            
            this.renderCameraPreviews(calibCameras);
        } catch (error) {
            console.error('Failed to load camera previews:', error);
        }
    },

    renderCameraPreviews(cameras) {
        const previewContainer = document.getElementById('calibration-preview');
        
        if (cameras.length === 0) {
            previewContainer.innerHTML = `
                <div class="col-12">
                    <div class="text-center py-4 text-muted">
                        <i class="bi bi-camera-video-off display-4"></i>
                        <p class="mt-2">No cameras in calibration mode</p>
                        <small>Set cameras to calibration mode in the Settings tab</small>
                    </div>
                </div>
            `;
            return;
        }

        previewContainer.innerHTML = cameras.map(camera => `
            <div class="col-md-6 col-lg-4">
                <div class="card">
                    <div class="card-body p-2">
                        <h6 class="card-title text-center mb-2">
                            <i class="bi bi-camera me-1"></i>
                            ${camera.id}
                        </h6>
                        <div class="camera-feed">
                            <img src="${window.CameraApp.API.getCameraStreamUrl(camera.det_port)}" 
                                 class="w-100 rounded" 
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
    },

    async startRecording() {
        const form = document.getElementById('calibration-params-form');
        const formData = window.CameraApp.UI.getFormData(form);
        const duration = formData.duration || 30;

        const startBtn = document.getElementById('start-recording-btn');
        const stopBtn = document.getElementById('stop-recording-btn');
        const statusEl = document.getElementById('recording-status');

        try {
            // Update UI
            window.CameraApp.UI.setButtonLoading(startBtn, true, 'Starting...');
            statusEl.style.display = 'block';
            statusEl.className = 'calibration-status warning';
            statusEl.innerHTML = 'Starting video recording...';

            // Start recording
            const response = await window.CameraApp.API.startRecording(duration);

            if (response.status === 'ok') {
                // Update UI for recording state
                startBtn.disabled = true;
                stopBtn.disabled = false;
                statusEl.className = 'calibration-status success';
                statusEl.innerHTML = `Recording ${response.started_cameras} cameras for ${duration}s...`;

                // Enable calibration button
                document.getElementById('start-calibration-btn').disabled = false;

                // Start countdown timer
                this.startRecordingCountdown(duration, response.started_cameras);

                window.CameraApp.UI.showToast(
                    `Recording started on ${response.started_cameras} cameras`,
                    'success'
                );
            } else {
                throw new Error(response.message || 'Recording failed');
            }
        } catch (error) {
            statusEl.className = 'calibration-status error';
            statusEl.innerHTML = `Recording failed: ${error.message}`;
            window.CameraApp.UI.showToast(`Recording failed: ${error.message}`, 'danger');
        } finally {
            window.CameraApp.UI.setButtonLoading(startBtn, false);
        }
    },

    startRecordingCountdown(duration, cameraCount) {
        const statusEl = document.getElementById('recording-status');
        let timeLeft = duration;

        const timer = setInterval(() => {
            timeLeft--;
            statusEl.innerHTML = `Recording: ${timeLeft}s remaining on ${cameraCount} cameras`;

            if (timeLeft <= 0) {
                clearInterval(timer);
                this.recordingCompleted();
            }
        }, 1000);

        // Store timer reference for manual stop
        this.recordingTimer = timer;
    },

    async stopRecording() {
        const stopBtn = document.getElementById('stop-recording-btn');
        
        try {
            window.CameraApp.UI.setButtonLoading(stopBtn, true, 'Stopping...');
            
            await window.CameraApp.API.stopRecording();
            
            // Clear countdown timer
            if (this.recordingTimer) {
                clearInterval(this.recordingTimer);
                this.recordingTimer = null;
            }
            
            this.recordingCompleted();
            window.CameraApp.UI.showToast('Recording stopped', 'info');
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to stop recording: ${error.message}`, 'danger');
        } finally {
            window.CameraApp.UI.setButtonLoading(stopBtn, false);
        }
    },

    recordingCompleted() {
        const startBtn = document.getElementById('start-recording-btn');
        const stopBtn = document.getElementById('stop-recording-btn');
        const statusEl = document.getElementById('recording-status');
        const calibBtn = document.getElementById('start-calibration-btn');

        // Reset recording UI
        startBtn.disabled = false;
        stopBtn.disabled = true;
        statusEl.className = 'calibration-status success';
        statusEl.innerHTML = 'Recording completed - ready for calibration';

        // Enable calibration
        calibBtn.disabled = false;
    },

    async startAutoCalibration() {
        const startBtn = document.getElementById('start-calibration-btn');
        const stopBtn = document.getElementById('stop-calibration-btn');
        const statusEl = document.getElementById('calibration-status');
        const progressEl = document.getElementById('calibration-progress');

        try {

            let camA = document.getElementById('camera-a-id').value.trim();
            let camB = document.getElementById('camera-b-id').value.trim();

//            const camA = document.getElementById('camera-a-id').value.trim();
//            const camB = document.getElementById('camera-b-id').value.trim();
            if (!camA || !camB) {
                const configured = await window.CameraApp.API.getConfiguredCameras();
                const ids = configured.map(c => c.id);
                const form = document.getElementById('calibration-params-form');
                const boardW = parseInt(form.board_cols.value) || window.CameraApp.Config.CALIBRATION.DEFAULT_BOARD_COLS;
                const boardH = parseInt(form.board_rows.value) || window.CameraApp.Config.CALIBRATION.DEFAULT_BOARD_ROWS;
                const analysis = await window.CameraApp.API.analyzeCompatibility(
                    ids,
                    boardW,
                    boardH,
                    window.CameraApp.Config.CALIBRATION.QUALITY_THRESHOLD,
                    window.CameraApp.Config.CALIBRATION.WIDE_ANGLE_THRESHOLD
                );
                if (analysis.suggested_pair && analysis.suggested_pair.length === 2) {
                    [camA, camB] = analysis.suggested_pair;
                    document.getElementById('camera-a-id').value = camA;
                    document.getElementById('camera-b-id').value = camB;
                } else {
                    throw new Error('Camera IDs required');
                }

            }

            // Save parameters first
            await this.saveCalibrationParams();

            // Update UI
            window.CameraApp.UI.setButtonLoading(startBtn, true, 'Starting...');
            stopBtn.disabled = false;
            statusEl.style.display = 'block';
            statusEl.className = 'calibration-status warning';
            statusEl.innerHTML = 'Starting automatic calibration...';
            progressEl.style.display = 'block';

            // Start calibration
            const response = await window.CameraApp.API.startAutoCalibration(camA, camB);

            if (response.status === 'ok') {
                statusEl.className = 'calibration-status success';
                statusEl.innerHTML = 'Calibration started - analyzing videos...';

                // Start progress monitoring
                this.startCalibrationMonitoring();
                // Initial status update
                this.getAutoCalibrationStatus();

                window.CameraApp.UI.showToast('Calibration started successfully', 'success');
            } else {
                throw new Error(response.error || 'Failed to start calibration');
            }
        } catch (error) {
            statusEl.className = 'calibration-status error';
            statusEl.innerHTML = `Calibration failed: ${error.message}`;
            progressEl.style.display = 'none';
            stopBtn.disabled = true;
            window.CameraApp.UI.showToast(`Calibration failed: ${error.message}`, 'danger');
        } finally {
            window.CameraApp.UI.setButtonLoading(startBtn, false);
        }
    },


    async getAutoCalibrationStatus() {
        try {
            const status = await window.CameraApp.API.getAutoCalibrationStatus();
            if (status.error) {
                throw new Error(status.error);
            }
            this.updateCalibrationProgress(status);
            return status;
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to get calibration status: ${error.message}`, 'danger');
            throw error;
        }
    },

    startCalibrationMonitoring() {
        this.statusTimer = setInterval(async () => {
            try {
                const status = await this.getAutoCalibrationStatus();
                if (!status.processing) {
                    clearInterval(this.statusTimer);
                    this.statusTimer = null;
                    this.calibrationCompleted(status);
                }
            } catch (error) {
                clearInterval(this.statusTimer);
                this.statusTimer = null;
            }
        }, 2000);
    },

    updateCalibrationProgress(status) {
        const statusEl = document.getElementById('calibration-status');
        const progressBar = document.querySelector('#calibration-progress .progress-bar');
        const progressText = document.getElementById('calibration-progress-text');

        statusEl.innerHTML = `${status.status_message} (${status.progress.toFixed(1)}%)`;
        progressBar.style.width = `${status.progress}%`;
        progressText.textContent = status.status_message;

    },

    async calibrationCompleted(status) {
        const startBtn = document.getElementById('start-calibration-btn');
        const stopBtn = document.getElementById('stop-calibration-btn');
        const statusEl = document.getElementById('calibration-status');
        const progressEl = document.getElementById('calibration-progress');

        // Reset UI
        startBtn.disabled = false;
        stopBtn.disabled = true;
        progressEl.style.display = 'none';

        if (status.mono_calibrations > 0) {
            statusEl.className = 'calibration-status success';
            let resultText = `Calibration SUCCESS! Found ${status.mono_calibrations} cameras`;
            if (status.stereo_calibrations > 0) {
                resultText += `, ${status.stereo_calibrations} stereo pairs`;
            }
            statusEl.innerHTML = resultText;

            // Show detailed results
            this.showCalibrationResults(status);

            // Copy results to working directory
            try {
                await window.CameraApp.API.copyCalibrationResults();
                window.CameraApp.UI.showToast('Calibration results copied to working directory', 'success');
            } catch (e) {
                console.error('Failed to copy results:', e);
            }
        } else {
            statusEl.className = 'calibration-status warning';
            statusEl.innerHTML = 'No calibration patterns found in videos';
            window.CameraApp.UI.showToast('No calibration patterns detected', 'warning');
        }
    },

    showCalibrationResults(status) {
        let resultsHtml = '<div class="mt-3"><strong>Results:</strong><br>';
        
        status.mono_results.forEach(result => {
            const icon = result.success ? 'check-circle' : 'x-circle';
            const color = result.success ? 'text-success' : 'text-danger';
            resultsHtml += `
                <div class="d-flex align-items-center mb-1">
                    <i class="bi bi-${icon} ${color} me-2"></i>
                    <span>${result.camera_id}: ${result.success ? 'OK' : 'FAILED'}</span>
                    ${result.success ? `<small class="text-muted ms-2">(error: ${result.reprojection_error.toFixed(2)})</small>` : ''}
                </div>
            `;
        });

        if (status.stereo_results.length > 0) {
            resultsHtml += '<br><strong>Stereo Pairs:</strong><br>';
            status.stereo_results.forEach(result => {
                const icon = result.success ? 'check-circle' : 'x-circle';
                const color = result.success ? 'text-success' : 'text-danger';
                resultsHtml += `
                    <div class="d-flex align-items-center mb-1">
                        <i class="bi bi-${icon} ${color} me-2"></i>
                        <span>${result.camera_pair}: ${result.success ? 'OK' : 'FAILED'}</span>
                        ${result.success ? `<small class="text-muted ms-2">(error: ${result.reprojection_error.toFixed(2)})</small>` : ''}
                    </div>
                `;
            });
        }

        resultsHtml += '</div>';
        
        const statusEl = document.getElementById('calibration-status');
        statusEl.innerHTML += resultsHtml;
    },

    async stopAutoCalibration() {
        try {
            await window.CameraApp.API.stopAutoCalibration();
            
            if (this.statusTimer) {
                clearInterval(this.statusTimer);
                this.statusTimer = null;
            }

            const stopBtn = document.getElementById('stop-calibration-btn');
            const statusEl = document.getElementById('calibration-status');
            const progressEl = document.getElementById('calibration-progress');

            stopBtn.disabled = true;
            statusEl.className = 'calibration-status warning';
            statusEl.innerHTML = 'Calibration stopped by user';
            progressEl.style.display = 'none';

            window.CameraApp.UI.showToast('Calibration stopped', 'info');
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to stop calibration: ${error.message}`, 'danger');
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

    stopRecordingTimer() {
        if (this.recordingTimer) {
            clearInterval(this.recordingTimer);
            this.recordingTimer = null;
        }
    },

    cleanup() {
        this.stopPreviewTimer();
        this.stopStatusTimer();
        this.stopRecordingTimer();
        
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
        this.startPreviewTimer();
    },

    deactivate() {
        this.cleanup();
        
        // Reset any ongoing operations
        if (this.currentCalibration) {
            this.stopAutoCalibration();
        }
    }
};