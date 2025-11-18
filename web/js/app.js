// Main Application Controller
window.CameraApp = window.CameraApp || {};

window.CameraApp.App = {
    currentTab: 'main-pane',

    async init() {
        console.log('Initializing Camera Manager Pro...');

        // Initialize UI components
        this.setupTabHandling();
        this.setupGlobalEventListeners();


        // Initialize shared modules that other tabs depend on
        // (e.g. MainTab calls GlobalTracker methods during startup)
        window.CameraApp.GlobalTracker.init();

        // Initialize tab modules
        await window.CameraApp.MainTab.init();
        await window.CameraApp.SettingsTab.init();
        await window.CameraApp.CalibrationTab.init();


        // Load initial configuration
        await this.loadInitialConfig();

        // Setup periodic status updates
        this.startStatusUpdates();

        // Initialize based on URL parameters
        this.handleInitialTab();

        console.log('Camera Manager Pro initialized successfully');
    },

    setupTabHandling() {
        const tabTriggerList = document.querySelectorAll('#mainTabs button[data-bs-toggle="tab"]');
        
        tabTriggerList.forEach(tabTrigger => {
            tabTrigger.addEventListener('shown.bs.tab', async (event) => {
                const newActiveTab = event.target.getAttribute('data-bs-target');
                const oldActiveTab = event.relatedTarget?.getAttribute('data-bs-target');
                
                console.log(`Tab changed from ${oldActiveTab} to ${newActiveTab}`);
                
                // Update URL without page reload
                const tabName = newActiveTab.replace('#', '').replace('-pane', '');
                this.updateURL(tabName);
                
                // Handle tab deactivation
                if (oldActiveTab) {
                    await this.deactivateTab(oldActiveTab);
                }
                
                // Handle tab activation
                await this.activateTab(newActiveTab);
                
                this.currentTab = newActiveTab;
            });
        });
    },

    setupGlobalEventListeners() {
        // Handle browser back/forward buttons
        window.addEventListener('popstate', this.handlePopState.bind(this));
        
        // Handle window focus/blur for performance optimization
        window.addEventListener('focus', this.handleWindowFocus.bind(this));
        window.addEventListener('blur', this.handleWindowBlur.bind(this));
        
        // Handle connection status
        window.addEventListener('online', this.handleOnline.bind(this));
        window.addEventListener('offline', this.handleOffline.bind(this));
        
        // Global error handler
        window.addEventListener('error', this.handleGlobalError.bind(this));
        window.addEventListener('unhandledrejection', this.handleUnhandledRejection.bind(this));
    },

    async loadInitialConfig() {
        try {
            const [status, config] = await Promise.all([
                window.CameraApp.API.getStatus(),
                window.CameraApp.API.getConfig()
            ]);

            window.CameraApp.State.previewEnabled = config.preview_enabled;

            const globalToggle = document.getElementById('global-tracking-toggle');
            const grayscaleToggle = document.getElementById('grayscale-tracking-toggle');

            const savedGlobal = !!(config.global_tracking ?? config.use_global_tracking);
            const savedGrayscale = !!config.grayscale_tracking;

            window.CameraApp.State.globalTrackingEnabled = savedGlobal;
            if (globalToggle) {
                globalToggle.checked = savedGlobal;
            }
            if (grayscaleToggle) {
                grayscaleToggle.checked = savedGrayscale;
            }

            window.CameraApp.State.managerDebugEnabled =
                !!config.manager_debug_enabled;

            if (window.CameraApp.State.globalTrackingEnabled) {
                window.CameraApp.State.lastGlobalFetch = 0;
                window.CameraApp.GlobalTracker.show();
                window.CameraApp.MainTab.startDetectionLoop();
            }

            window.CameraApp.UI.updateStatusIndicator('online', 'Connected');

            console.log('Initial configuration loaded:', { status, config });
        } catch (error) {
            console.error('Failed to load initial configuration:', error);
            window.CameraApp.UI.updateStatusIndicator('error', 'Connection Error');
            window.CameraApp.UI.showToast('Failed to connect to server', 'danger');
        }
    },

    handleInitialTab() {
        const urlParams = new URLSearchParams(window.location.search);
        const tabParam = urlParams.get('tab') || 'main';
        
        // Activate the appropriate tab
        const tabButton = document.querySelector(`#${tabParam}-tab`);
        if (tabButton) {
            const tab = new bootstrap.Tab(tabButton);
            tab.show();
        }
    },

    updateURL(tabName) {
        const url = new URL(window.location);
        url.searchParams.set('tab', tabName);
        window.history.pushState({ tab: tabName }, '', url);
    },

    handlePopState(event) {
        if (event.state && event.state.tab) {
            const tabButton = document.querySelector(`#${event.state.tab}-tab`);
            if (tabButton) {
                const tab = new bootstrap.Tab(tabButton);
                tab.show();
            }
        }
    },

    async activateTab(tabId) {
        switch (tabId) {
            case '#main-pane':
                await window.CameraApp.MainTab.activate();
                break;
            case '#settings-pane':
                await window.CameraApp.SettingsTab.activate();
                break;
            case '#calibration-pane':
                await window.CameraApp.CalibrationTab.activate();
                break;
        }
    },

    async deactivateTab(tabId) {
        switch (tabId) {
            case '#main-pane':
                window.CameraApp.MainTab.deactivate();
                break;
            case '#settings-pane':
                window.CameraApp.SettingsTab.deactivate();
                break;
            case '#calibration-pane':
                window.CameraApp.CalibrationTab.deactivate();
                break;
        }
    },

    startStatusUpdates() {
        // Update status every 5 seconds
        setInterval(async () => {
            try {
                const status = await window.CameraApp.API.getStatus();
                window.CameraApp.UI.updateStatusIndicator('online', 'Connected');
            } catch (error) {
                window.CameraApp.UI.updateStatusIndicator('error', 'Connection Lost');
            }
        }, 5000);
    },

    handleWindowFocus() {
        console.log('Window focused - resuming updates');
        // Resume any paused operations
    },

    handleWindowBlur() {
        console.log('Window blurred - optimizing performance');
        // Pause non-critical operations
    },

    handleOnline() {
        window.CameraApp.UI.updateStatusIndicator('online', 'Connected');
        window.CameraApp.UI.showToast('Connection restored', 'success');
    },

    handleOffline() {
        window.CameraApp.UI.updateStatusIndicator('error', 'Offline');
        window.CameraApp.UI.showToast('Connection lost', 'warning');
    },

    handleGlobalError(event) {
        console.error('Global error:', event.error);
        window.CameraApp.UI.showToast('An unexpected error occurred', 'danger');
    },

    handleUnhandledRejection(event) {
        console.error('Unhandled promise rejection:', event.reason);
        window.CameraApp.UI.showToast('An unexpected error occurred', 'danger');
        event.preventDefault(); // Prevent the error from being logged to console
    },

    // Utility methods for other modules
    gotoTab(tabName) {
        const tabButton = document.querySelector(`#${tabName}-tab`);
        if (tabButton) {
            const tab = new bootstrap.Tab(tabButton);
            tab.show();
        }
    },

    // Cleanup method for page unload
    cleanup() {
        // Stop all timers and cleanup resources
        window.CameraApp.MainTab.deactivate();
        window.CameraApp.SettingsTab.deactivate();
        window.CameraApp.CalibrationTab.deactivate();
    }
};

// Initialize the application when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.CameraApp.App.init().catch(error => {
        console.error('Failed to initialize application:', error);
        document.body.innerHTML = `
            <div class="container mt-5">
                <div class="alert alert-danger text-center">
                    <h4>Failed to Initialize</h4>
                    <p>The application failed to start. Please refresh the page or check the console for details.</p>
                    <button class="btn btn-danger" onclick="location.reload()">Reload Page</button>
                </div>
            </div>
        `;
    });
});

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    window.CameraApp.App.cleanup();
});

// Export for global access
window.CameraApp.gotoTab = window.CameraApp.App.gotoTab.bind(window.CameraApp.App);
