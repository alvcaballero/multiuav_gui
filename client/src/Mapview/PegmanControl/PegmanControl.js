/**
 * MapLibre 3D View Control
 * A MapLibre GL JS control that allows users to navigate to a 3D view by dragging a pegman
 */

import './styles.css';
// based on https://github.com/rezw4n/maplibre-google-streetview/tree/master
import { useEffect, useMemo } from 'react';
import { useDispatch } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import { map } from '../MapView'; // Access the global map instance
import { sessionActions } from '../../store';

/**
 * Maplibre3DViewControl class implementing IControl interface
 * @class
 */
class Maplibre3DViewControl {
  /**
   * Create a new Maplibre3DViewControl instance
   * @param {Object} options - Configuration options
   * @param {Function} options.navigate - Navigation function (e.g., navigate('/3Dview'))
   * @param {Function} options.dispatch - Redux dispatch function
   * @param {number} [options.defaultAltitude=100] - Default altitude for 3D view
   */
  constructor(options = {}) {
    this.navigate = options.navigate;
    this.dispatch = options.dispatch;
    this.defaultAltitude = options.defaultAltitude || 100;

    this.map = null;
    this.container = null;
    this.isDragging = false;
    this.pegmanMarker = null;

    // Touch interaction tracking
    this.touchStartTime = 0;
    this.touchMoved = false;
    this.isTouchInteraction = false;
  }

  /**
   * Called when the control is added to the map
   * @param {Object} map - MapLibre map instance
   * @returns {HTMLElement} The control's container element
   */
  onAdd(map) {
    this.map = map;

    // Create pegman container
    this.container = document.createElement('div');
    this.container.className = 'mapboxgl-ctrl mapboxgl-ctrl-group maplibre-streetview-pegman-container';

    // Create pegman button element
    this.pegman = document.createElement('button');
    this.pegman.className = 'maplibre-streetview-pegman';
    this.pegman.type = 'button';
    this.pegman.setAttribute('aria-label', '3D View');
    this.pegman.setAttribute('draggable', 'true');
    this.container.appendChild(this.pegman);

    // Create pegman marker (appears when dragging)
    this._createPegmanMarker();

    // Set up event listeners
    this._setupEventListeners();

    return this.container;
  }

  /**
   * Called when the control is removed from the map
   */
  onRemove() {
    // Remove pegman marker
    if (this.pegmanMarker && this.pegmanMarker.parentNode) {
      this.pegmanMarker.parentNode.removeChild(this.pegmanMarker);
    }

    // Remove event listeners
    if (this.container && this.container.parentNode) {
      this.container.parentNode.removeChild(this.container);
    }

    this.map = null;
  }

  /**
   * Create the pegman marker that appears when dragging
   * @private
   */
  _createPegmanMarker() {
    if (this.pegmanMarker) return;

    this.pegmanMarker = document.createElement('div');
    this.pegmanMarker.className = 'maplibre-streetview-pegman-marker';
    document.body.appendChild(this.pegmanMarker);
  }

  /**
   * Set up event listeners
   * @private
   */
  _setupEventListeners() {
    // Make pegman draggable
    this.pegman.addEventListener('dragstart', (e) => {
      console.log('dragstart');
      this.isDragging = true;
      this.pegman.classList.add('dragging');
      this.container.classList.add('dragging');

      // Make the drag image transparent
      const emptyImg = new Image();
      emptyImg.src = 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7';
      e.dataTransfer.setDragImage(emptyImg, 0, 0);
      e.dataTransfer.effectAllowed = 'move';

      // Show and position the pegman marker at the cursor
      this.pegmanMarker.classList.add('active');
      this._updatePegmanMarkerPosition(e);
    });

    // Handle pegman dragging over map
    this.map.getContainer().addEventListener('dragover', (e) => {
      console.log('dragover');
      e.preventDefault();
      if (this.isDragging) {
        this._updatePegmanMarkerPosition(e);
        e.dataTransfer.dropEffect = 'move';
      }
    });

    // Handle mousemove for updating pegman marker position
    document.addEventListener('mousemove', (e) => {
      if (this.isDragging) {
        this._updatePegmanMarkerPosition(e);
      }
    });

    // Handle pegman drop on map
    this.map.getContainer().addEventListener('drop', (e) => {
      console.log('drop')
      e.preventDefault();

      if (this.isDragging) {
        const rect = this.map.getContainer().getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;

        // Convert drop point to geographic coordinates
        const lngLat = this.map.unproject([x, y]);

        // Dispatch Redux action to update 3D scene origin
        if (this.dispatch ) {
          this.dispatch(sessionActions.updateScene3dOrigin({
            lat: lngLat.lat,
            lng: lngLat.lng,
            alt: this.defaultAltitude
          }));
        }

        // Navigate to 3D view
        if (this.navigate) {
          this.navigate('/3Dview');
        }
      }

      // Reset dragging state
      this._endDragging();
    });

    // End drag if it leaves the map area
    this.map.getContainer().addEventListener('dragleave', (e) => {
      console.log('dragleave');
      if (this.isDragging) {
        // Only hide marker if we're actually leaving the map container
        // Check if the related target (where we're going) is outside the map container
        const mapContainer = this.map.getContainer();
        const relatedTarget = e.relatedTarget;

        if (!relatedTarget || !mapContainer.contains(relatedTarget)) {
          this.pegmanMarker.classList.remove('active');
        }
      }
    });

    // Handle dragenter to show marker again when re-entering map
    this.map.getContainer().addEventListener('dragenter', (e) => {
      console.log('dragenter');
      if (this.isDragging) {
        // Only show marker if we're entering from outside the map container
        const mapContainer = this.map.getContainer();
        const relatedTarget = e.relatedTarget;

        if (!relatedTarget || !mapContainer.contains(relatedTarget)) {
          this.pegmanMarker.classList.add('active');
        }
      }
    });

    // Handle the end of dragging
    this.pegman.addEventListener('dragend', () => {
      console.log('dragend');
      this._endDragging();
    });

    // MOBILE TOUCH SUPPORT
    // Handle touchstart on pegman for mobile drag
    this.pegman.addEventListener('touchstart', (e) => {
      e.preventDefault();

      // Track touch start to distinguish between tap and drag
      this.touchStartTime = Date.now();
      this.touchMoved = false;
      this.isTouchInteraction = true;
    }, { passive: false });

    // Handle touchmove for updating pegman marker position
    document.addEventListener('touchmove', (e) => {
      if (this.isTouchInteraction) {
        this.touchMoved = true;

        // Start dragging if we haven't already
        if (!this.isDragging) {
          this.isDragging = true;
          this.pegman.classList.add('dragging');
          this.container.classList.add('dragging');
          this.pegmanMarker.classList.add('active');
        }

        // Prevent page scrolling while dragging
        e.preventDefault();

        const touch = e.touches[0];
        if (touch) {
          this._updatePegmanMarkerPosition(touch);
        }
      }
    }, { passive: false });

    // Handle touchend (drop) on map
    document.addEventListener('touchend', (e) => {
      if (this.isTouchInteraction) {
        const touchDuration = Date.now() - this.touchStartTime;
        const touch = e.changedTouches[0];

        // If it was a drag (not just a tap)
        if (this.touchMoved && this.isDragging && touch) {
          const mapContainer = this.map.getContainer();
          const rect = mapContainer.getBoundingClientRect();

          // Check if touch ended inside the map container
          if (touch.clientX >= rect.left && touch.clientX <= rect.right &&
              touch.clientY >= rect.top && touch.clientY <= rect.bottom) {

            // Convert touch point to map coordinates
            const x = touch.clientX - rect.left;
            const y = touch.clientY - rect.top;

            // Convert to geographic coordinates
            const lngLat = this.map.unproject([x, y]);

            // Dispatch Redux action to update 3D scene origin
            if (this.dispatch && this.sessionActions) {
              this.dispatch(this.sessionActions.updateScene3dOrigin({
                lat: lngLat.lat,
                lng: lngLat.lng,
                alt: this.defaultAltitude
              }));
            }

            // Navigate to 3D view
            if (this.navigate) {
              this.navigate('/3Dview');
            }
          }
        }

        // Reset touch and drag states
        this._endDragging();
        this.touchMoved = false;
        this.isTouchInteraction = false;
      }
    }, { passive: false });
  }

  /**
   * Update pegman marker position during drag
   * @private
   * @param {Event|Touch} e - Mouse event or Touch object
   */
  _updatePegmanMarkerPosition(e) {
    if (!this.isDragging || !this.pegmanMarker) return;

    // Support both mouse events and touch objects
    const clientX = e.clientX !== undefined ? e.clientX : e.pageX;
    const clientY = e.clientY !== undefined ? e.clientY : e.pageY;

    this.pegmanMarker.style.left = (clientX - 10) + 'px';
    this.pegmanMarker.style.top = (clientY - 30) + 'px';
  }

  /**
   * End dragging state
   * @private
   */
  _endDragging() {
    this.isDragging = false;
    this.pegman.classList.remove('dragging');
    this.container.classList.remove('dragging');

    if (this.pegmanMarker) {
      this.pegmanMarker.classList.remove('active');
    }

    // Reset touch states
    this.touchMoved = false;
    this.isTouchInteraction = false;
  }
}


const PegmanControl = () => {
    // Create the control instance once
    const navigate = useNavigate();
    const dispatch = useDispatch();
    const control = useMemo(() => new Maplibre3DViewControl({ navigate, dispatch }), []);

    useEffect(() => {
        map.addControl(control, 'top-right');
        return () => { map.removeControl(control); };
    }, [control]);

    // Render the button into the control's container using a Portal
    return null;
};

export default PegmanControl;


