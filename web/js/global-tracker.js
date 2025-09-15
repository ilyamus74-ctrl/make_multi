// Global Tracker Module
window.CameraApp = window.CameraApp || {};

window.CameraApp.GlobalTracker = {
    panel: null,
    
    init() {
        this.panel = document.getElementById('global-tracker-panel');
        this.setupEventListeners();
    },

    setupEventListeners() {
        // Close button should disable global tracking
        const closeBtn = document.getElementById('close-tracker-panel');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => {
                this.hide();
                const toggle = document.getElementById('global-tracking-toggle');
                if (toggle) toggle.checked = false;
                window.CameraApp.State.globalTrackingEnabled = false;
                // Stop detection loop in MainTab
                if (window.CameraApp.MainTab?.stopDetectionLoop) {
                    window.CameraApp.MainTab.stopDetectionLoop();
                }
            });
        }
    },

    show() {
        if (this.panel) {
            this.panel.style.display = 'block';
            window.CameraApp.UI.fadeIn(this.panel);
        }
    },

    hide() {
        if (this.panel) {
            window.CameraApp.UI.fadeOut(this.panel);
        }
    },

    showMessage(message) {
        const content = document.getElementById('global-tracker-content');
        if (content) {
            content.innerHTML = `<div class="text-center text-muted">${message}</div>`;
        }
    },


    async refresh() {
        try {
            const objects = await window.CameraApp.API.getGlobalTracking();
            window.CameraApp.State.lastGlobalObjects = objects;
            this.updateContent(objects);
        } catch (error) {
            console.error('Failed to retrieve global tracking data:', error);
        }
    },


    updateContent(objects) {
        const content = document.getElementById('global-tracker-content');
        if (!content) return;

        if (!objects || objects.length === 0) {
            content.innerHTML = `
                <div class="text-center text-muted py-3">
                    <i class="bi bi-search display-4"></i>
                    <p class="mt-2 mb-0">No objects detected</p>
                </div>
            `;
            return;
        }

        // Sort objects by class name, then by ID
        const sortedObjects = [...objects].sort((a, b) => {
            const nameA = a.class_name || 'Unknown';
            const nameB = b.class_name || 'Unknown';
            if (nameA !== nameB) return nameA.localeCompare(nameB);
            return a.id - b.id;
        });

        content.innerHTML = sortedObjects.map(obj => 
            window.CameraApp.UI.createTrackedObjectCard(
                obj, 
                obj.id === window.CameraApp.State.selectedObjectId
            )
        ).join('');

        // Add click handlers
        content.querySelectorAll('.tracked-object').forEach(element => {
            element.addEventListener('click', () => {
                const objectId = parseInt(element.dataset.objectId);
                this.selectObject(objectId);
            });
        });
    },

    selectObject(objectId) {
        const wasSelected = window.CameraApp.State.selectedObjectId === objectId;
        window.CameraApp.State.selectedObjectId = wasSelected ? -1 : objectId;

        // Update UI immediately
        this.updateContent(window.CameraApp.State.lastGlobalObjects);

        // Highlight object on camera streams
        this.highlightObjectOnStreams(wasSelected ? -1 : objectId);

        console.log('Selected object ID:', window.CameraApp.State.selectedObjectId);
    },

    async highlightObjectOnStreams(objectId) {
        const promises = window.CameraApp.State.mainCameras.map(async (camera) => {
            try {
                await window.CameraApp.API.highlightObject(camera.det_port, objectId);
            } catch (error) {
                console.error(`Failed to highlight object on camera ${camera.id}:`, error);
            }
        });

        await Promise.allSettled(promises);
    }
};