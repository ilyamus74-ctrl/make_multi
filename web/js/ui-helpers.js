// UI Helper Functions
window.CameraApp = window.CameraApp || {};

window.CameraApp.UI = {
    // Toast notifications
    showToast(message, type = 'info', duration = 3000) {
        const toastContainer = this.getToastContainer();
        const toastId = window.CameraApp.Utils.generateId();
        
        const toastHtml = `
            <div class="toast align-items-center text-bg-${type} border-0" role="alert" id="toast-${toastId}">
                <div class="d-flex">
                    <div class="toast-body">
                        <i class="bi bi-${this.getToastIcon(type)} me-2"></i>
                        ${message}
                    </div>
                    <button type="button" class="btn-close btn-close-white me-2 m-auto" data-bs-dismiss="toast"></button>
                </div>
            </div>
        `;
        
        toastContainer.insertAdjacentHTML('beforeend', toastHtml);
        
        const toastElement = document.getElementById(`toast-${toastId}`);
        const toast = new bootstrap.Toast(toastElement, { delay: duration });
        toast.show();
        
        // Remove from DOM after hiding
        toastElement.addEventListener('hidden.bs.toast', () => {
            toastElement.remove();
        });
    },

    getToastContainer() {
        let container = document.getElementById('toast-container');
        if (!container) {
            container = document.createElement('div');
            container.id = 'toast-container';
            container.className = 'toast-container position-fixed top-0 end-0 p-3';
            container.style.zIndex = '1055';
            document.body.appendChild(container);
        }
        return container;
    },

    getToastIcon(type) {
        const icons = {
            success: 'check-circle',
            danger: 'exclamation-triangle',
            warning: 'exclamation-circle',
            info: 'info-circle'
        };
        return icons[type] || 'info-circle';
    },

    // Modal management
    showModal(title, content, options = {}) {
        const modalId = `modal-${window.CameraApp.Utils.generateId()}`;
        const size = options.size || '';
        const showFooter = options.showFooter !== false;
        
        const modalHtml = `
            <div class="modal fade" id="${modalId}" tabindex="-1">
                <div class="modal-dialog ${size}">
                    <div class="modal-content">
                        <div class="modal-header">
                            <h5 class="modal-title">${title}</h5>
                            <button type="button" class="btn-close" data-bs-dismiss="modal"></button>
                        </div>
                        <div class="modal-body">
                            ${content}
                        </div>
                        ${showFooter ? `
                            <div class="modal-footer">
                                <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Close</button>
                            </div>
                        ` : ''}
                    </div>
                </div>
            </div>
        `;
        
        document.getElementById('modals-container').insertAdjacentHTML('beforeend', modalHtml);
        
        const modal = new bootstrap.Modal(document.getElementById(modalId));
        modal.show();
        
        // Cleanup after hiding
        document.getElementById(modalId).addEventListener('hidden.bs.modal', () => {
            document.getElementById(modalId).remove();
        });
        
        return modal;
    },

    // Confirm dialog
    async confirm(message, title = 'Confirm Action') {
        return new Promise((resolve) => {
            const content = `
                <p>${message}</p>
                <div class="d-flex justify-content-end gap-2">
                    <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Cancel</button>
                    <button type="button" class="btn btn-danger" id="confirm-btn">Confirm</button>
                </div>
            `;
            
            const modal = this.showModal(title, content, { showFooter: false });
            
            // Add event listeners
            modal._element.querySelector('#confirm-btn').addEventListener('click', () => {
                modal.hide();
                resolve(true);
            });
            
            modal._element.addEventListener('hidden.bs.modal', () => {
                resolve(false);
            });
        });
    },

    // Progress modal
    showProgressModal(title, initialMessage = 'Starting...') {
        const modalId = `progress-modal-${window.CameraApp.Utils.generateId()}`;
        
        const modalHtml = `
            <div class="modal fade" id="${modalId}" tabindex="-1" data-bs-backdrop="static" data-bs-keyboard="false">
                <div class="modal-dialog">
                    <div class="modal-content">
                        <div class="modal-header">
                            <h5 class="modal-title">${title}</h5>
                        </div>
                        <div class="modal-body">
                            <div class="progress mb-3">
                                <div class="progress-bar" role="progressbar" style="width: 0%"></div>
                            </div>
                            <div class="progress-message">${initialMessage}</div>
                        </div>
                    </div>
                </div>
            </div>
        `;
        
        document.getElementById('modals-container').insertAdjacentHTML('beforeend', modalHtml);
        
        const modal = new bootstrap.Modal(document.getElementById(modalId));
        modal.show();
        
        return {
            modal,
            updateProgress: (percent, message) => {
                const progressBar = modal._element.querySelector('.progress-bar');
                const messageEl = modal._element.querySelector('.progress-message');
                progressBar.style.width = `${percent}%`;
                progressBar.setAttribute('aria-valuenow', percent);
                if (message) messageEl.textContent = message;
            },
            close: () => {
                modal.hide();
                setTimeout(() => modal._element.remove(), 300);
            }
        };
    },

    // Status indicator
    updateStatusIndicator(status, text) {
        const indicator = document.getElementById('status-indicator');
        const statusText = document.getElementById('status-text');
        const icon = indicator.querySelector('i');
        
        // Remove old status classes
        icon.className = 'bi me-1';
        
        // Set new status
        switch(status) {
            case 'online':
                icon.classList.add('bi-circle-fill', 'text-success');
                break;
            case 'warning':
                icon.classList.add('bi-exclamation-triangle-fill', 'text-warning');
                break;
            case 'error':
                icon.classList.add('bi-x-circle-fill', 'text-danger');
                break;
            default:
                icon.classList.add('bi-circle-fill', 'text-secondary');
        }
        
        statusText.textContent = text;
    },

    // Loading state management
    setLoadingState(element, isLoading, loadingText = 'Loading...') {
        if (isLoading) {
            element.dataset.originalContent = element.innerHTML;
            element.innerHTML = window.CameraApp.Utils.createLoadingElement(loadingText);
            element.disabled = true;
        } else {
            if (element.dataset.originalContent) {
                element.innerHTML = element.dataset.originalContent;
                delete element.dataset.originalContent;
            }
            element.disabled = false;
        }
    },

    // Button loading state
    setButtonLoading(button, isLoading, loadingText = 'Loading...') {
        if (isLoading) {
            button.dataset.originalText = button.textContent;
            button.innerHTML = `<span class="loading-spinner me-2"></span>${loadingText}`;
            button.disabled = true;
        } else {
            if (button.dataset.originalText) {
                button.textContent = button.dataset.originalText;
                delete button.dataset.originalText;
            }
            button.disabled = false;
        }
    },

    // Create camera preview element
    createCameraPreview(camera, options = {}) {
        const showControls = options.showControls !== false;
        const showFps = options.showFps !== false;
        const size = options.size || 'normal';
        
        const sizeClasses = {
            small: 'col-md-4 col-lg-3',
            normal: 'col-md-6 col-lg-4',
            large: 'col-md-8 col-lg-6'
        };
        
        return `
            <div class="${sizeClasses[size]} mb-3" data-camera-id="${camera.id}">
                <div class="card camera-card ${camera.present ? 'active' : 'inactive'}">
                    <div class="card-body">
                        <h6 class="card-title d-flex align-items-center justify-content-between">
                            <span>
                                <i class="bi bi-camera me-2"></i>
                                ${camera.id}
                            </span>
                            ${showFps && camera.show_fps ? `<span class="fps-badge">FPS: ...</span>` : ''}
                        </h6>
                        
                        <div class="camera-preview-container mb-2">
                            ${this.createCameraPreviewImage(camera)}
                        </div>
                        
                        ${showControls ? this.createCameraControls(camera) : ''}
                    </div>
                </div>
            </div>
        `;
    },

    createCameraPreviewImage(camera) {
        if (!window.CameraApp.State.previewEnabled || !camera.present) {
            return `
                <div class="camera-feed-overlay">
                    <div class="text-center">
                        <i class="bi bi-camera-video-off display-4"></i>
                        <p class="mt-2 mb-0">${camera.present ? 'Preview disabled' : 'Camera offline'}</p>
                    </div>
                </div>
            `;
        }
        
        let imageUrl;
        if (camera.mode === 'preview') {
            imageUrl = window.CameraApp.API.getPreviewMjpgUrl(camera.id);
        } else if (camera.mode === 'detect') {
            imageUrl = window.CameraApp.API.getCameraStreamUrl(camera.det_port);
        } else if (camera.mode === 'calibration') {
            imageUrl = window.CameraApp.API.getCameraStreamUrl(camera.det_port);
        }
        
        return `
            <div class="camera-feed">
                <img src="${imageUrl}" class="camera-preview" alt="Camera ${camera.id}">
                <div class="camera-feed-label">${camera.mode.toUpperCase()}</div>
            </div>
        `;
    },

    createCameraControls(camera) {
        return `
            <div class="camera-controls">
                <select class="form-select form-select-sm me-2" data-camera-mode="${camera.id}">
                    <option value="preview" ${camera.mode === 'preview' ? 'selected' : ''}>Preview</option>
                    <option value="detect" ${camera.mode === 'detect' ? 'selected' : ''}>Detect</option>
                    <option value="calibration" ${camera.mode === 'calibration' ? 'selected' : ''}>Calibration</option>
                </select>


                <select class="form-select form-select-sm me-2" data-camera-role="${camera.id}">
                    ${window.CameraApp.Config.CAMERA.ROLES.map(r => `
                        <option value="${r}" ${camera.role === r ? 'selected' : ''}>${r}</option>`).join('')}
                </select>

                
                <button class="btn btn-outline-primary btn-sm me-2" data-camera-settings="${camera.id}">
                    <i class="bi bi-gear"></i>
                </button>
                
                <button class="btn btn-outline-danger btn-sm" data-camera-delete="${camera.id}">
                    <i class="bi bi-trash"></i>
                </button>
            </div>
        `;
    },

    // Create new camera card
    createNewCameraCard(camera, index = 0) {
        const identifiers = Array.isArray(camera.identifiers) && camera.identifiers.length > 0
            ? camera.identifiers
            : [{ type: 'device', value: camera.device || '' }];

        const utils = window.CameraApp && window.CameraApp.Utils;
        const isHdmiInput = utils && typeof utils.isHdmiInput === 'function'
            ? utils.isHdmiInput(camera)
            : false;

        const preferredIndex = Math.max(0, identifiers.findIndex(id => id.type === 'by-id' && id.value));
        const defaultIdentifier = identifiers[preferredIndex] || identifiers[0];

        const identifierOptions = identifiers.map((identifier, optionIndex) => `
            <option value="${optionIndex}" ${optionIndex === preferredIndex ? 'selected' : ''}>
                ${identifier.type}: ${identifier.value}
            </option>
        `).join('');

        const identifierSelector = identifiers.length > 1
            ? `
                <div class="mb-2">
                    <select class="form-select form-select-sm" data-new-camera-select>
                        ${identifierOptions}
                    </select>
                </div>
            `
            : `
                <div class="mb-2">
                    <small class="text-muted text-monospace">${defaultIdentifier.type}: ${defaultIdentifier.value}</small>
                </div>
            `;

        const infoParts = [];
        if (isHdmiInput) {
            infoParts.push(`
                <div class="alert alert-warning small text-start mb-2">
                    <div class="fw-semibold"><i class="bi bi-plug me-1"></i>HDMI capture device</div>
                    <div class="mb-0">Requires an active HDMI signal before it can stream.</div>
                </div>
            `);
        }
        if (camera.card) {
            infoParts.push(`<div class="fw-semibold">${camera.card}</div>`);
        }
        if (camera.device) {
            infoParts.push(`<div class="text-muted small text-break">${camera.device}</div>`);
        }
        if (camera.bus_info) {
            infoParts.push(`<div class="text-muted small text-break">Bus: ${camera.bus_info}</div>`);
        }
        const infoBlock = infoParts.length ? `<div class="mb-2">${infoParts.join('')}</div>` : '';

        return `
                <div class="card new-camera-card" data-new-camera-index="${index}">
                <div class="card new-camera-card">
                    <div class="card-body text-center">
                        <h6 class="card-title">
                            <i class="bi bi-camera-plus me-2"></i>
                            New Camera
                        </h6>

                        <div class="camera-preview-container mb-3">
                            <img class="camera-preview" data-new-camera data-new-camera-type="${defaultIdentifier.type}"
                                 data-new-camera-value="${defaultIdentifier.value}" alt="New camera preview">
                        </div>


                        ${infoBlock}
                        ${identifierSelector}

                        <button
                            class="btn btn-primary btn-sm"
                            data-add-camera="${defaultIdentifier.value}"
                            data-match-type="${defaultIdentifier.type}"
                            data-match-value="${defaultIdentifier.value}">
                            <i class="bi bi-plus me-1"></i>Add Camera
                        </button>
                    </div>
                </div>
            </div>
        `;
    },

    // Global tracker object card
    createTrackedObjectCard(obj, isSelected = false) {
        const classInfo = window.CameraApp.Utils.getObjectClassInfo(obj.class_id);


        const distanceParts = [];
        if (typeof obj.distance_mono_m === 'number' && Number.isFinite(obj.distance_mono_m)) {
            distanceParts.push(`Mono: ${obj.distance_mono_m.toFixed(2)} m`);
        }
        if (typeof obj.distance_stereo_m === 'number' && Number.isFinite(obj.distance_stereo_m)) {
            distanceParts.push(`Stereo: ${obj.distance_stereo_m.toFixed(2)} m`);
        }
        if (distanceParts.length === 0 && typeof obj.distance_world_m === 'number' && Number.isFinite(obj.distance_world_m)) {
            distanceParts.push(`${obj.distance_world_m.toFixed(2)} m`);
        }

        const distanceBlock = distanceParts.length > 0 ? `
                <div class="small">
                    <i class="bi bi-rulers me-1"></i>
                    Distance: ${distanceParts.join(' ')}
                </div>
            ` : '';

        return `
            <div class="tracked-object ${isSelected ? 'selected' : ''}" data-object-id="${obj.id}">
                <div class="d-flex align-items-start justify-content-between mb-2">
                    <div>
                        <span class="object-badge" style="background-color: ${classInfo.color}; color: ${classInfo.textColor};">
                            ${obj.class_name || classInfo.name}
                        </span>
                        <strong>Object #${obj.id}</strong>
                    </div>
                    <small class="text-muted">${(obj.confidence * 100).toFixed(1)}%</small>
                </div>
                
                <div class="small text-muted mb-2">
                    <i class="bi bi-camera me-1"></i>
                    ${obj.cameras.map(c => c.camera).join(', ')}
                </div>
                ${distanceBlock}
            </div>
        `;
    },

    // Animation helpers
    fadeIn(element, duration = 300) {
        element.style.opacity = '0';
        element.style.display = 'block';
        
        setTimeout(() => {
            element.style.transition = `opacity ${duration}ms ease-in-out`;
            element.style.opacity = '1';
        }, 10);
    },

    fadeOut(element, duration = 300) {
        element.style.transition = `opacity ${duration}ms ease-in-out`;
        element.style.opacity = '0';
        
        setTimeout(() => {
            element.style.display = 'none';
        }, duration);
    },

    // Form helpers
    getFormData(form) {
        const formData = new FormData(form);
        const data = {};
        
        for (let [key, value] of formData.entries()) {
            // Handle checkboxes
            if (form.querySelector(`[name="${key}"]`).type === 'checkbox') {
                data[key] = form.querySelector(`[name="${key}"]`).checked;
            }
            // Handle numbers
            else if (form.querySelector(`[name="${key}"]`).type === 'number') {
                data[key] = parseFloat(value) || 0;
            }
            // Handle regular inputs
            else {
                data[key] = value;
            }
        }
        
        return data;
    },

    // Validation helpers
    validateForm(form) {
        const inputs = form.querySelectorAll('input[required], select[required], textarea[required]');
        let isValid = true;
        
        inputs.forEach(input => {
            if (!input.value.trim()) {
                input.classList.add('is-invalid');
                isValid = false;
            } else {
                input.classList.remove('is-invalid');
            }
        });
        
        return isValid;
    }
};