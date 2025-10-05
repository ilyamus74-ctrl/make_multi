
// Main Tab (Live Stream) Module
window.CameraApp = window.CameraApp || {};



window.CameraApp.MainTab = {
    detectionTimer: null,
    
    async init() {
        this.setupEventListeners();
        await this.loadCameras();
        await this.loadTrackingModes();
    },

    setupEventListeners() {
        // Global tracking toggle
        const globalToggle = document.getElementById('global-tracking-toggle');
        globalToggle.addEventListener('change', this.handleGlobalTrackingToggle.bind(this));

        // Grayscale tracking toggle
        const grayscaleToggle = document.getElementById('grayscale-tracking-toggle');
        grayscaleToggle.addEventListener('change', this.handleGrayscaleTrackingToggle.bind(this));

        // Clear memory button
        const clearBtn = document.getElementById('clear-memory-btn');
        clearBtn.addEventListener('click', this.handleClearMemory.bind(this));
    },

    async handleGlobalTrackingToggle(event) {
        const enabled = event.target.checked;
        window.CameraApp.State.globalTrackingEnabled = enabled;

        try {
            await window.CameraApp.API.setTrackingMode(enabled);
            
            if (enabled) {
                window.CameraApp.GlobalTracker.show();
                this.startDetectionLoop();
            } else {
                window.CameraApp.GlobalTracker.hide();
                this.stopDetectionLoop();
            }

            window.CameraApp.UI.showToast(
                `Global tracking ${enabled ? 'enabled' : 'disabled'}`,
                'success'
            );
        } catch (error) {
            window.CameraApp.UI.showToast('Failed to toggle tracking mode', 'danger');
            event.target.checked = !enabled; // Revert toggle
        }
    },


    async handleGrayscaleTrackingToggle(event) {
        const enabled = event.target.checked;

        try {
            await window.CameraApp.API.setGrayscaleTrackingMode(enabled);

            window.CameraApp.UI.showToast(
                `Grayscale tracking mode ${enabled ? 'enabled' : 'disabled'}`,
                'success'
            );
        } catch (error) {
            console.error('Failed to toggle grayscale tracking mode:', error);
            window.CameraApp.UI.showToast('Failed to toggle grayscale tracking mode', 'danger');
            event.target.checked = !enabled; // Revert toggle
        }
    },



    handleClearMemory() {
        window.CameraApp.State.selectedObjectId = -1;
        window.CameraApp.State.lastGlobalObjects = [];
        
        // Clear global tracker display
        window.CameraApp.GlobalTracker.showMessage('Memory cleared');

        // Force garbage collection if available
        if (window.gc) window.gc();
        
        window.CameraApp.UI.showToast('Memory cleared', 'info');
    },

    async loadTrackingModes() {
        try {
            // Load grayscale tracking mode state
            const grayscaleState = await window.CameraApp.API.getGrayscaleTrackingMode();
            const grayscaleToggle = document.getElementById('grayscale-tracking-toggle');
            if (grayscaleToggle && grayscaleState.status === 'ok') {
                grayscaleToggle.checked = grayscaleState.grayscale_tracking || false;
            }
        } catch (error) {
            console.error('Failed to load tracking modes:', error);
        }
    },


    async loadCameras() {
        try {
            const cameras = await window.CameraApp.API.getConfiguredCameras();
            window.CameraApp.State.mainCameras = cameras.filter(c => 
                c.mode === 'detect' && c.det_running
            );
            
            this.renderVideoGrid();
            
            if (window.CameraApp.State.globalTrackingEnabled) {
                this.startDetectionLoop();
            }
        } catch (error) {
            console.error('Failed to load cameras:', error);
            this.showError('Failed to load camera list');
        }
    },

    renderVideoGrid() {
        const mosaic = document.getElementById('video-mosaic');
        const noCameras = document.getElementById('no-cameras');
        const cameras = window.CameraApp.State.mainCameras;

        if (cameras.length === 0) {
            mosaic.style.display = 'none';
            noCameras.style.display = 'block';
            return;
        }

        mosaic.style.display = 'block';
        noCameras.style.display = 'none';

        // Create grid layout
        mosaic.innerHTML = cameras.map(camera => `
            <div class="col-md-6 col-lg-4 col-xl-3">
                <div class="card h-100">
                    <div class="card-body p-2">
                        <h6 class="card-title text-center mb-2">
                            <i class="bi bi-camera-video me-1"></i>
                            ${camera.id}
                        </h6>
                        <div class="camera-feed">
                            <img src="${window.CameraApp.API.getCameraStreamUrl(camera.det_port)}" 
                                 class="w-100 rounded" 
                                 alt="Camera ${camera.id}"
                                 style="height: 200px; object-fit: contain;">
                            <div class="camera-feed-label">DETECT</div>
                        </div>
                    </div>
                </div>
            </div>
        `).join('');
    },

    startDetectionLoop() {
        if (this.detectionTimer) {
            clearInterval(this.detectionTimer);
        }

        this.detectionTimer = setInterval(() => {
            this.fetchDetections();
        }, window.CameraApp.Config.UI.GLOBAL_TRACKING_INTERVAL);

        // Initial fetch
        this.fetchDetections();
    },

    stopDetectionLoop() {
        if (this.detectionTimer) {
            clearInterval(this.detectionTimer);
            this.detectionTimer = null;
        }
    },

    async fetchDetections() {
        if (!window.CameraApp.State.globalTrackingEnabled) {
            return;
        }

        // Rate limiting
        const now = Date.now();
        if (now - window.CameraApp.State.lastGlobalFetch < window.CameraApp.Config.UI.GLOBAL_TRACKING_INTERVAL) {
            return;
        }
        window.CameraApp.State.lastGlobalFetch = now;

        try {
            await window.CameraApp.API.updateDetections();
            await window.CameraApp.GlobalTracker.refresh();
        } catch (error) {

            console.error('Failed to fetch global detections:', error);
        }
    },



    showError(message) {
        const mosaic = document.getElementById('video-mosaic');
        mosaic.innerHTML = window.CameraApp.Utils.createErrorElement(message, true);
    },

    // Tab activation/deactivation
    async activate() {
        await window.CameraApp.API.enablePreview(true);
        window.CameraApp.State.previewEnabled = true;
        await this.loadCameras();
    },

    deactivate() {
        this.stopDetectionLoop();
        window.CameraApp.GlobalTracker.hide();
        
        // Reset global tracking state
        window.CameraApp.State.globalTrackingEnabled = false;
        window.CameraApp.State.selectedObjectId = -1;
        window.CameraApp.State.lastGlobalObjects = [];
        
        const globalToggle = document.getElementById('global-tracking-toggle');
        globalToggle.checked = false;
    }
};
