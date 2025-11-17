
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
            const config = await window.CameraApp.API.getConfig();
            const globalToggle = document.getElementById('global-tracking-toggle');
            const savedGlobal = !!(config.global_tracking ?? config.use_global_tracking);
            window.CameraApp.State.globalTrackingEnabled = savedGlobal;
            if (globalToggle) {
                globalToggle.checked = savedGlobal;
            }
            if (savedGlobal) {
                window.CameraApp.GlobalTracker.show();
                this.startDetectionLoop();
            }
            // Load grayscale tracking mode state
            const grayscaleState = await window.CameraApp.API.getGrayscaleTrackingMode();
            const grayscaleToggle = document.getElementById('grayscale-tracking-toggle');
            const grayscaleEnabled = grayscaleState.status === 'ok'
                ? grayscaleState.grayscale_tracking
                : config.grayscale_tracking;
            if (grayscaleToggle) {
                grayscaleToggle.checked = !!grayscaleEnabled;
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

        const selections = window.CameraApp.State.mosaicSelections || [null, null, null, null];

        const availableIds = cameras.map(c => c.id);
        const assigned = new Set();
        selections.forEach((sel, idx) => {
            if (sel && !availableIds.includes(sel)) {
                selections[idx] = null;
            }
            if (selections[idx]) {
                assigned.add(selections[idx]);
            }
        });

        let cursor = 0;
        selections.forEach((sel, idx) => {
            if (!sel && cameras[cursor]) {
                while (cursor < cameras.length && assigned.has(cameras[cursor].id)) {
                    cursor++;
                }
                if (cameras[cursor]) {
                    selections[idx] = cameras[cursor].id;
                    assigned.add(cameras[cursor].id);
                    cursor++;
                }
            }
        });

        window.CameraApp.State.mosaicSelections = selections;

        if (cameras.length === 0) {
            mosaic.style.display = 'none';
            noCameras.style.display = 'block';
            return;
        }

        mosaic.style.display = 'block';
        noCameras.style.display = 'none';

        const slotTemplate = (slotIndex) => {
            const selectedId = selections[slotIndex];
            const camera = cameras.find(c => c.id === selectedId);
            const options = [
                `<option value="">Select camera</option>`,
                ...cameras.map(c => `<option value="${c.id}" ${c.id === selectedId ? 'selected' : ''}>${c.id}</option>`)
            ].join('');

            const content = camera
                ? `<img class="camera-stream" src="${window.CameraApp.API.getCameraStreamUrl(camera.det_port)}" alt="Camera ${camera.id}">
                   <div class="camera-feed-label">DETECT</div>`
                : `<div class="camera-placeholder">EMPTY</div>`;

            return `
                <div class="card mosaic-card">
                    <div class="card-body d-flex flex-column gap-2">
                        <div class="d-flex justify-content-between align-items-center">
                            <div class="d-flex align-items-center gap-2">
                                <i class="bi bi-grid-3x3-gap"></i>
                                <span class="fw-semibold">${slotIndex + 1}</span>
                            </div>
                            <select class="form-select form-select-sm mosaic-select" data-slot="${slotIndex}">
                                ${options}
                            </select>
                        </div>
                        <div class="camera-feed flex-grow-1">${content}</div>
                    </div>
                </div>`;
        };

        mosaic.innerHTML = [0, 1, 2, 3].map(slotTemplate).join('');

        mosaic.querySelectorAll('.mosaic-select').forEach(select => {
            select.addEventListener('change', (event) => {
                const slot = parseInt(event.target.dataset.slot, 10);
                const value = event.target.value || null;
                window.CameraApp.State.mosaicSelections[slot] = value;
                this.renderVideoGrid();
            });
        });
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

        if (window.CameraApp.State.globalTrackingEnabled) {
            window.CameraApp.GlobalTracker.show();
            this.startDetectionLoop();
        }
    },

    deactivate() {
        this.stopDetectionLoop();
        window.CameraApp.GlobalTracker.hide();

        // Reset global tracking state
//        window.CameraApp.State.globalTrackingEnabled = false;
//        window.CameraApp.State.selectedObjectId = -1;
//        window.CameraApp.State.lastGlobalObjects = [];

//        const globalToggle = document.getElementById('global-tracking-toggle');
//        globalToggle.checked = false;
    }
};
