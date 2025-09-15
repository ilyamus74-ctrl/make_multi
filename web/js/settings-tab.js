// Settings Tab Module
window.CameraApp = window.CameraApp || {};

window.CameraApp.SettingsTab = {
    refreshTimer: null,
    
    async init() {
        this.setupEventListeners();
        await this.loadData();
    },

    setupEventListeners() {
        // Stereo runtime button
        document.getElementById('stereo-runtime-btn').addEventListener('click', 
            this.showStereoRuntimeModal.bind(this));

        // Event delegation for dynamic elements
        document.addEventListener('click', this.handleDynamicEvents.bind(this));
        document.addEventListener('change', this.handleDynamicChanges.bind(this));
    },

    handleDynamicEvents(event) {
        const target = event.target;

        // Camera settings button
        if (target.matches('[data-camera-settings]')) {
            const cameraId = target.getAttribute('data-camera-settings');
            this.showAdvancedSettings(cameraId);
        }

        // Camera delete button
        if (target.matches('[data-camera-delete]')) {
            const cameraId = target.getAttribute('data-camera-delete');
            this.deleteCamera(cameraId);
        }

        // Add new camera button
        if (target.matches('[data-add-camera]')) {
            const byIdPath = target.getAttribute('data-add-camera');
            this.addNewCamera(byIdPath);
        }

        // Advanced settings buttons
        if (target.id === 'save-advanced-settings') {
            this.saveAdvancedSettings();
        }

        if (target.id === 'reset-advanced-settings') {
            this.resetAdvancedSettings();
        }
    },

    handleDynamicChanges(event) {
        const target = event.target;

        // Camera mode change
        if (target.matches('[data-camera-mode]')) {
            const cameraId = target.getAttribute('data-camera-mode');
            const mode = target.value;
            this.changeCameraMode(cameraId, mode);
        }

        // Camera role change
        if (target.matches('[data-camera-role]')) {
            const cameraId = target.getAttribute('data-camera-role');
            const role = target.value;
            this.changeCameraRole(cameraId, role);
        }
    },

    async loadData() {
        try {
            const [configuredCameras, newCameras, roles] = await Promise.all([
                window.CameraApp.API.getConfiguredCameras(),
                window.CameraApp.API.getNewCameras(),
                window.CameraApp.API.getCameraRoles()
            ]);

            // merge roles info into configured cameras
            roles.forEach(r => {
                const cam = configuredCameras.find(c => c.id === r.id);
                if (cam) cam.role = r.role;
            });


            window.CameraApp.State.configuredCameras = configuredCameras;
            window.CameraApp.State.newCameras = newCameras;

            window.CameraApp.State.cameraRoles = roles;
            this.renderConfiguredCameras();
            this.renderNewCameras();
        } catch (error) {
            console.error('Failed to load camera data:', error);
            this.showError('Failed to load camera information');
        }
    },

    renderConfiguredCameras() {
        const container = document.getElementById('configured-cameras');
        const cameras = window.CameraApp.State.configuredCameras;

        if (cameras.length === 0) {
            container.innerHTML = `
                <div class="text-center py-4 text-muted">
                    <i class="bi bi-camera display-4"></i>
                    <p class="mt-2">No cameras configured</p>
                </div>
            `;
            return;
        }

        container.innerHTML = `
            <div class="row">
                ${cameras.map(camera => window.CameraApp.UI.createCameraPreview(camera, {
                    showControls: true,
                    showFps: true,
                    size: 'normal'
                })).join('')}
            </div>
        `;

        // Load camera preview images with delay
        this.loadCameraPreviewsWithDelay();

        // Load FPS data for cameras that have show_fps enabled
        this.loadCameraFpsData();
    },

    renderNewCameras() {
        const container = document.getElementById('new-cameras');
        const noCamerasEl = document.getElementById('no-new-cameras');
        const cameras = window.CameraApp.State.newCameras;

        if (cameras.length === 0) {
            container.innerHTML = '';
            noCamerasEl.style.display = 'block';
            return;
        }

        noCamerasEl.style.display = 'none';
        container.innerHTML = `
            <div class="row">
                ${cameras.map((camera, index) => 
                    window.CameraApp.UI.createNewCameraCard(camera, index)
                ).join('')}
            </div>
        `;

        // Load new camera previews with delay
        this.loadNewCameraPreviewsWithDelay();
    },

    loadCameraPreviewsWithDelay() {
        const images = document.querySelectorAll('.camera-preview');
        images.forEach((img, index) => {
            setTimeout(() => {
                const cameraCard = img.closest('[data-camera-id]');
                if (cameraCard && window.CameraApp.State.previewEnabled) {
                    const cameraId = cameraCard.getAttribute('data-camera-id');
                    const camera = window.CameraApp.State.configuredCameras.find(c => c.id === cameraId);
                    
                    if (camera && camera.present) {
                        img.src = this.getCameraImageUrl(camera);
                        img.onerror = () => {
                            img.style.display = 'none';
                            img.parentElement.innerHTML = `
                                <div class="camera-feed-overlay">
                                    <div class="text-center">
                                        <i class="bi bi-exclamation-triangle text-warning"></i>
                                        <p class="mt-1 mb-0">Preview Error</p>
                                    </div>
                                </div>
                            `;
                        }
                    };
                }
            }, index * 200);
        });
    },

    loadNewCameraPreviewsWithDelay() {
        const images = document.querySelectorAll('[data-new-camera]');
        images.forEach((img, index) => {
            setTimeout(() => {
                const byIdPath = img.getAttribute('data-new-camera');
                if (window.CameraApp.State.previewEnabled) {
                    img.src = window.CameraApp.API.getNewCameraPreviewUrl(byIdPath);
                    img.onerror = () => {
                        img.style.display = 'none';
                        img.parentElement.innerHTML = `
                            <div class="camera-feed-overlay">
                                <div class="text-center">
                                    <i class="bi bi-exclamation-triangle text-warning"></i>
                                    <p class="mt-1 mb-0">Preview Error</p>
                                </div>
                            </div>
                        `;
                    };
                }
            }, index * 200);
        });
    },


    loadCameraFpsData() {
        const cameras = window.CameraApp.State.configuredCameras;
        cameras.forEach(async (camera, index) => {
            if (camera.show_fps && camera.det_running) {
                setTimeout(async () => {
                    try {
                        const health = await window.CameraApp.API.getCameraHealth(camera.det_port);
                        const fpsBadge = document.querySelector(`[data-camera-id="${camera.id}"] .fps-badge`);
                        if (fpsBadge && health.cap_real_fps) {
                            fpsBadge.textContent = `${health.cap_real_fps.toFixed(1)} FPS`;
                        }
                    } catch (e) {
                        console.error(`Failed to get FPS for ${camera.id}:`, e);
                        const fpsBadge = document.querySelector(`[data-camera-id="${camera.id}"] .fps-badge`);
                        if (fpsBadge) {
                            fpsBadge.textContent = 'FPS: --';
                        }
                    }
                }, index * 100);
            }
        });
    },

/*    getCameraImageUrl(camera) {
        if (camera.mode === 'preview') {
            return window.CameraApp.API.getPreviewMjpgUrl(camera.id);
        } else if (camera.mode === 'detect') {
            return window.CameraApp.API.getCameraStreamUrl(camera.det_port);
        } else if (camera.mode === 'calibration') {
            return window.CameraApp.API.getCameraStreamUrl(camera.det_port);
        }
        return '';
    },

    getCameraImageUrl(camera) {
        if (camera.mode === 'preview') {
            return `/api/preview.mjpg?id=${encodeURIComponent(camera.id)}`;
        } else if (camera.mode === 'detect') {
            return `${location.protocol}//${location.hostname}:${camera.det_port}/api/stream.mjpg`;
        } else if (camera.mode === 'calibration') {
            return `${location.protocol}//${location.hostname}:${camera.det_port}/api/stream.mjpg`;
        }
        return '';
    },*/
    getCameraImageUrl(camera) {
    const timestamp = Date.now(); // Cache busting
       if (camera.mode === 'preview') {
           // Preview использует процесс детекции с флагом --no-draw
           return `${location.protocol}//${location.hostname}:${camera.det_port}/api/stream.mjpg`;
       } else if (camera.mode === 'detect') {
           // Detect использует главный сервер для проксирования
           return `/api/preview.mjpg?id=${encodeURIComponent(camera.id)}`;
       } else if (camera.mode === 'calibration') {
           return `${location.protocol}//${location.hostname}:${camera.det_port}/api/stream.mjpg`;
       }
        return '';
     },

    async changeCameraMode(cameraId, mode) {
    try {
        await window.CameraApp.API.setCameraMode(cameraId, mode);
        window.CameraApp.UI.showToast(`Camera ${cameraId} mode changed to ${mode}`, 'success');
        
        // Показываем "загрузка" пока процессы перезапускаются
        const cameraCard = document.querySelector(`[data-camera-id="${cameraId}"]`);
        if (cameraCard) {
            const img = cameraCard.querySelector('.camera-preview');
            if (img && img.parentElement) {
                img.src = ''; // Очищаем старое изображение
                img.style.display = 'none';
                img.parentElement.innerHTML = `
                    <div class="camera-feed-overlay">
                        <div class="text-center">
                            <div class="loading-spinner"></div>
                            <p class="mt-1 mb-0">Switching mode...</p>
                        </div>
                    </div>
                `;
            }
        }
        // Увеличиваем задержку для полной перезагрузки процессов
        setTimeout(() => this.loadData(), 3000);
    } catch (error) {
            window.CameraApp.UI.showToast(`Failed to change camera mode: ${error.message}`, 'danger');
            // Revert the select
            const select = document.querySelector(`[data-camera-mode="${cameraId}"]`);
            if (select) {
                const camera = window.CameraApp.State.configuredCameras.find(c => c.id === cameraId);
                if (camera) select.value = camera.mode;
            }
        }
    },

    async changeCameraRole(cameraId, role) {
        try {
            await window.CameraApp.API.setCameraRole(cameraId, role);
            window.CameraApp.UI.showToast(`Camera ${cameraId} role set to ${role}`, 'success');
            const cam = window.CameraApp.State.configuredCameras.find(c => c.id === cameraId);
            if (cam) cam.role = role;
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to change camera role: ${error.message}`, 'danger');
            const select = document.querySelector(`[data-camera-role="${cameraId}"]`);
            const cam = window.CameraApp.State.configuredCameras.find(c => c.id === cameraId);
            if (select && cam) select.value = cam.role;
        }
    },


    async deleteCamera(cameraId) {
        const confirmed = await window.CameraApp.UI.confirm(
            `Are you sure you want to delete camera "${cameraId}"? This action cannot be undone.`,
            'Delete Camera'
        );

        if (!confirmed) return;

        try {
            await window.CameraApp.API.deleteCamera(cameraId);
            window.CameraApp.UI.showToast(`Camera ${cameraId} deleted successfully`, 'success');
            await this.loadData();
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to delete camera: ${error.message}`, 'danger');
        }
    },

    async addNewCamera(byIdPath) {
        const cameraId = prompt('Enter camera ID (letters, numbers, underscore, and dash only):');
        
        if (!cameraId) return;
        
        if (!window.CameraApp.Utils.validateCameraId(cameraId)) {
            window.CameraApp.UI.showToast('Invalid camera ID format', 'danger');
            return;
        }

        try {
            await window.CameraApp.API.addCamera(cameraId, byIdPath);
            window.CameraApp.UI.showToast(`Camera ${cameraId} added successfully`, 'success');
            await this.loadData();
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to add camera: ${error.message}`, 'danger');
        }
    },

    async showAdvancedSettings(cameraId) {
        const camera = window.CameraApp.State.configuredCameras.find(c => c.id === cameraId);
        if (!camera) return;

        window.CameraApp.State.currentAdvancedCameraId = cameraId;
        
        try {
            const models = await window.CameraApp.API.getModels();
            this.renderAdvancedSettings(camera, models);
            
            const panel = document.getElementById('advanced-settings');
            panel.style.display = 'block';
            window.CameraApp.UI.fadeIn(panel);
            window.CameraApp.State.advancedSettingsVisible = true;
        } catch (error) {
            window.CameraApp.UI.showToast('Failed to load advanced settings', 'danger');
        }
    },

    renderAdvancedSettings(camera, models) {
        const content = document.getElementById('advanced-settings-content');
        
        content.innerHTML = `
            <form id="advanced-settings-form">
                <div class="setting-group">
                    <label class="form-label">Model</label>
                    <select name="model_path" class="form-select form-select-sm">
                        ${models.rknn.map(model => 
                            `<option value="${model}" ${camera.model_path === model ? 'selected' : ''}>${model}</option>`
                        ).join('')}
                    </select>
                </div>

                <div class="setting-group">
                    <label class="form-label">Labels</label>
                    <select name="labels_path" class="form-select form-select-sm">
                        ${models.labels.map(labels => 
                            `<option value="${labels}" ${camera.labels_path === labels ? 'selected' : ''}>${labels}</option>`
                        ).join('')}
                    </select>
                </div>

                <div class="setting-group">
                    <label class="form-label">Video Settings</label>
                    <div class="row g-2">
                        <div class="col-6">
                            <label class="form-label small">Width</label>
                            <input type="number" name="preferred_w" class="form-control form-control-sm" 
                                   value="${camera.preferred.w}" min="320" max="1920">
                        </div>
                        <div class="col-6">
                            <label class="form-label small">Height</label>
                            <input type="number" name="preferred_h" class="form-control form-control-sm" 
                                   value="${camera.preferred.h}" min="240" max="1080">
                        </div>
                        <div class="col-6">
                            <label class="form-label small">Format</label>
                            <input type="text" name="preferred_pixfmt" class="form-control form-control-sm" 
                                   value="${camera.preferred.pixfmt}">
                        </div>
                        <div class="col-6">
                            <label class="form-label small">FPS</label>
                            <input type="number" name="preferred_fps" class="form-control form-control-sm" 
                                   value="${camera.preferred.fps}" min="1" max="60">
                        </div>
                    </div>
                </div>

                <div class="setting-group">
                    <label class="form-label">Performance</label>
                    <div class="row g-2">
                        <div class="col-6">
                            <label class="form-label small">NPU Worker</label>
                            <input type="number" name="npu_worker" class="form-control form-control-sm" 
                                   value="${camera.npu_worker}" min="0" max="2">
                        </div>
                        <div class="col-6">
                            <label class="form-label small">NPU Core</label>
                            <select name="npu_core" class="form-select form-select-sm">
                                <option value="auto" ${camera.npu_core === 'auto' ? 'selected' : ''}>Auto</option>
                                <option value="0" ${camera.npu_core === '0' ? 'selected' : ''}>Core 0</option>
                                <option value="1" ${camera.npu_core === '1' ? 'selected' : ''}>Core 1</option>
                                <option value="2" ${camera.npu_core === '2' ? 'selected' : ''}>Core 2</option>
                                <option value="01" ${camera.npu_core === '01' ? 'selected' : ''}>Core 0+1</option>
                                <option value="012" ${camera.npu_core === '012' ? 'selected' : ''}>All Cores</option>
                            </select>
                        </div>
                    </div>
                </div>

                <div class="setting-group">
                    <div class="form-check">
                        <input class="form-check-input" type="checkbox" name="auto_profiles" 
                               ${camera.auto_profiles ? 'checked' : ''}>
                        <label class="form-check-label">Auto Profiles</label>
                    </div>
                    
                    <label class="form-label small mt-2">Profile</label>
                    <select name="profile" class="form-select form-select-sm">
                        ${window.CameraApp.Config.CAMERA.PROFILES.map(profile => 
                            `<option value="${profile}" ${camera.profile === profile ? 'selected' : ''}>${profile}</option>`
                        ).join('')}
                    </select>
                </div>

                <div class="setting-group">
                    <label class="form-label">Advanced</label>
                    <div class="row g-2">
                        <div class="col-6">
                            <label class="form-label small">Capture FPS</label>
                            <input type="number" name="cap_fps" class="form-control form-control-sm" 
                                   value="${camera.cap_fps}" min="1" max="60">
                        </div>
                        <div class="col-6">
                            <label class="form-label small">Buffers</label>
                            <input type="number" name="buffers" class="form-control form-control-sm" 
                                   value="${camera.buffers}" min="1" max="8">
                        </div>
                        <div class="col-6">
                            <label class="form-label small">Buffer Type</label>
                            <select name="buffer_type" class="form-select form-select-sm">
                                ${window.CameraApp.Config.CAMERA.BUFFER_TYPES.map(type => 
                                    `<option value="${type}" ${camera.buffer_type === type ? 'selected' : ''}>${type}</option>`
                                ).join('')}
                            </select>
                        </div>
                        <div class="col-6">
                            <label class="form-label small">JPEG Quality</label>
                            <input type="number" name="jpeg_quality" class="form-control form-control-sm" 
                                   value="${camera.jpeg_quality}" min="30" max="95">
                        </div>
                        <div class="col-6">
                            <label class="form-label small">HTTP FPS Limit</label>
                            <input type="number" name="http_fps_limit" class="form-control form-control-sm" 
                                   value="${camera.http_fps_limit}" min="0" max="60">
                        </div>
                    </div>
                    
                    <div class="form-check mt-2">
                        <input class="form-check-input" type="checkbox" name="show_fps" 
                               ${camera.show_fps ? 'checked' : ''}>
                        <label class="form-check-label">Show FPS</label>
                    </div>
                    
                    <label class="form-label small mt-2">Log File</label>
                    <input type="text" name="log_file" class="form-control form-control-sm" 
                           value="${camera.log_file}" placeholder="Optional log file path">
                </div>

                <div class="d-flex gap-2 mt-3">
                    <button type="button" class="btn btn-primary btn-sm flex-fill" id="save-advanced-settings">
                        <i class="bi bi-check me-1"></i>Save
                    </button>
                    <button type="button" class="btn btn-outline-secondary btn-sm flex-fill" id="reset-advanced-settings">
                        <i class="bi bi-arrow-counterclockwise me-1"></i>Reset
                    </button>
                </div>
            </form>
        `;
    },

    async saveAdvancedSettings() {
        const form = document.getElementById('advanced-settings-form');
        const cameraId = window.CameraApp.State.currentAdvancedCameraId;
        
        if (!window.CameraApp.UI.validateForm(form)) {
            window.CameraApp.UI.showToast('Please fill in all required fields', 'warning');
            return;
        }

        const saveBtn = document.getElementById('save-advanced-settings');
        window.CameraApp.UI.setButtonLoading(saveBtn, true, 'Saving...');

        try {
            const formData = window.CameraApp.UI.getFormData(form);
            
            const settings = {
                preferred: {
                    w: formData.preferred_w,
                    h: formData.preferred_h,
                    pixfmt: formData.preferred_pixfmt,
                    fps: formData.preferred_fps
                },
                npu_worker: formData.npu_worker,
                auto_profiles: formData.auto_profiles,
                profile: formData.profile,
                model_path: formData.model_path,
                labels_path: formData.labels_path,
                cap_fps: formData.cap_fps,
                buffers: formData.buffers,
                buffer_type: formData.buffer_type,
                jpeg_quality: formData.jpeg_quality,
                http_fps_limit: formData.http_fps_limit,
                show_fps: formData.show_fps,
                npu_core: formData.npu_core,
                log_file: formData.log_file
            };

            await window.CameraApp.API.updateCameraSettings(cameraId, settings);
            window.CameraApp.UI.showToast('Settings saved successfully', 'success');
            
            // Refresh camera data
            await this.loadData();
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to save settings: ${error.message}`, 'danger');
        } finally {
            window.CameraApp.UI.setButtonLoading(saveBtn, false);
        }
    },

    async resetAdvancedSettings() {
        const confirmed = await window.CameraApp.UI.confirm(
            'Reset all settings to default values?',
            'Reset Settings'
        );

        if (!confirmed) return;

        const cameraId = window.CameraApp.State.currentAdvancedCameraId;
        const resetBtn = document.getElementById('reset-advanced-settings');
        window.CameraApp.UI.setButtonLoading(resetBtn, true, 'Resetting...');

        try {
            await window.CameraApp.API.resetCameraSettings(cameraId);
            window.CameraApp.UI.showToast('Settings reset to defaults', 'success');
            
            // Refresh and reload advanced settings
            await this.loadData();
            const camera = window.CameraApp.State.configuredCameras.find(c => c.id === cameraId);
            if (camera) {
                const models = await window.CameraApp.API.getModels();
                this.renderAdvancedSettings(camera, models);
            }
        } catch (error) {
            window.CameraApp.UI.showToast(`Failed to reset settings: ${error.message}`, 'danger');
        } finally {
            window.CameraApp.UI.setButtonLoading(resetBtn, false);
        }
    },

    showStereoRuntimeModal() {
        const content = `
            <form id="stereo-runtime-form">
                <div class="mb-3">
                    <label class="form-label">Detection Range (meters)</label>
                    <input type="number" name="range" class="form-control" value="0" min="0" max="100">
                    <div class="form-text">Maximum distance for object detection (0 = unlimited)</div>
                </div>
                
                <div class="form-check mb-3">
                    <input class="form-check-input" type="checkbox" name="tracking">
                    <label class="form-check-label">Enable object tracking</label>
                </div>
                
                <div class="form-check mb-3">
                    <input class="form-check-input" type="checkbox" name="global_tracking">
                    <label class="form-check-label">Global multi-camera tracking</label>
                </div>
                
                <div class="d-flex justify-content-end gap-2">
                    <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Cancel</button>
                    <button type="submit" class="btn btn-primary">Save Settings</button>
                </div>
            </form>
        `;

        const modal = window.CameraApp.UI.showModal('Stereo Runtime Settings', content, { 
            showFooter: false,
            size: 'modal-lg'
        });

        // Handle form submission
        const form = modal._element.querySelector('#stereo-runtime-form');
        form.addEventListener('submit', async (e) => {
            e.preventDefault();
            const formData = window.CameraApp.UI.getFormData(form);
            
            try {
                // Save stereo runtime configuration
                console.log('Stereo runtime settings:', formData);
                window.CameraApp.UI.showToast('Stereo settings saved', 'success');
                modal.hide();
            } catch (error) {
                window.CameraApp.UI.showToast('Failed to save settings', 'danger');
            }
        });
    },

    showError(message) {
        const container = document.getElementById('configured-cameras');
        container.innerHTML = window.CameraApp.Utils.createErrorElement(message, true);
    },

    startRefreshTimer() {
        this.refreshTimer = setInterval(() => {
            // Более умное обновление - проверяем нужно ли обновлять
            this.smartRefresh();
        }, window.CameraApp.Config.UI.REFRESH_INTERVAL);
    },

    async smartRefresh() {
        try {
            // Получаем свежие данные без перерисовки UI
            const [configuredCameras, newCameras] = await Promise.all([
                window.CameraApp.API.getConfiguredCameras(),
                window.CameraApp.API.getNewCameras()
            ]);

            // Проверяем нужно ли обновлять configured cameras
            const needConfiguredUpdate = this.needsUpdate(
                window.CameraApp.State.configuredCameras, 
                configuredCameras
            );

            // Проверяем нужно ли обновлять new cameras  
            const needNewUpdate = this.needsUpdate(
                window.CameraApp.State.newCameras, 
                newCameras
            );

            // Обновляем состояние
            window.CameraApp.State.configuredCameras = configuredCameras;
            window.CameraApp.State.newCameras = newCameras;

            // Обновляем UI только если действительно нужно
            if (needConfiguredUpdate) {
                this.renderConfiguredCameras();
            }

            if (needNewUpdate) {
                this.renderNewCameras();
            }

        } catch (error) {
            console.error('Failed to refresh camera data:', error);
        }
    },

    needsUpdate(oldData, newData) {
        // Быстрая проверка - если количество изменилось
        if (oldData.length !== newData.length) return true;

        // Проверяем изменения в ключевых полях
        for (let i = 0; i < oldData.length; i++) {
            const oldItem = oldData[i];
            const newItem = newData[i];

            // Для камер проверяем критичные изменения
            if (oldItem.id && newItem.id) {
                if (oldItem.present !== newItem.present ||
                    oldItem.det_running !== newItem.det_running ||
                    Math.abs((oldItem.fps || 0) - (newItem.fps || 0)) > 0.5) {
                    return true;
                }
            }
        }

        return false;
    },

    stopRefreshTimer() {
        if (this.refreshTimer) {
            clearInterval(this.refreshTimer);
            this.refreshTimer = null;
        }
    },

    // Tab activation/deactivation
    async activate() {
        await window.CameraApp.API.enablePreview(true);
        window.CameraApp.State.previewEnabled = true;
        await this.loadData();
        this.startRefreshTimer();
        
        // Hide advanced settings panel if switching tabs
        if (window.CameraApp.State.advancedSettingsVisible) {
            const panel = document.getElementById('advanced-settings');
            panel.style.display = 'none';
            window.CameraApp.State.advancedSettingsVisible = false;
            window.CameraApp.State.currentAdvancedCameraId = null;
        }
    },

    deactivate() {
        this.stopRefreshTimer();
        
        // Hide advanced settings panel
        if (window.CameraApp.State.advancedSettingsVisible) {
            const panel = document.getElementById('advanced-settings');
            panel.style.display = 'none';
            window.CameraApp.State.advancedSettingsVisible = false;
            window.CameraApp.State.currentAdvancedCameraId = null;
        }
    }
};
