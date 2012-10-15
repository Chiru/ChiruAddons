/**
 * @author Eberhard Graether / http://egraether.com/
 */

THREE.TrackballControls = function ( object, domElement ) {

    THREE.EventTarget.call( this );

    var _this = this,
        STATE = { NONE : -1, ROTATE : 0, ZOOM : 1, PAN : 2 };

    this.object = object;
    this.domElement = ( domElement !== undefined ) ? domElement : document;

    // API

    this.enabled = true;

    this.screen = { width: 0, height: 0, offsetLeft: 0, offsetTop: 0 };
    this.radius = ( this.screen.width + this.screen.height ) / 4;

    this.rotateSpeed = 1.0;
    this.zoomSpeed = 1.2;
    this.panSpeed = 0.3;

    this.noRotate = false;
    this.noZoom = false;
    this.noPan = false;

    this.staticMoving = false;
    this.dynamicDampingFactor = 0.2;

    this.minDistance = 0;
    this.maxDistance = Infinity;
    this.pinchZoomTriggerDist = 0.03;

    this.keys = [ 65 /*A*/, 83 /*S*/, 68 /*D*/ ];

    // internals

    this.target = new THREE.Vector3();

    var lastPosition = new THREE.Vector3();

    var _keyPressed = false,
        _state = STATE.NONE,
        _initialTouchDist = 0,
        _touchDist = 0,
        _twoFingers = false,
        _touchStart = [],
        _touchEnd = [],
        _startDistance = 0,


        _eye = new THREE.Vector3(),

        _rotateStart = new THREE.Vector3(),
        _rotateEnd = new THREE.Vector3(),

        _zoomStart = new THREE.Vector2(),
        _zoomEnd = new THREE.Vector2(),

        _panStart = new THREE.Vector2(),
        _panEnd = new THREE.Vector2();

    // events

    var changeEvent = { type: 'change' };


    // methods

    this.handleResize = function () {

        this.screen.width = window.innerWidth;
        this.screen.height = window.innerHeight;

        this.screen.offsetLeft = 0;
        this.screen.offsetTop = 0;

        this.radius = ( this.screen.width + this.screen.height ) / 4;
    };

    this.handleEvent = function ( event ) {

        if ( typeof this[ event.type ] == 'function' ) {

            this[ event.type ]( event );

        }

    };

    this.getMouseOnScreen = function ( clientX, clientY ) {

        return new THREE.Vector2(
            ( clientX - _this.screen.offsetLeft ) / _this.radius * 0.5,
            ( clientY - _this.screen.offsetTop ) / _this.radius * 0.5
        );

    };

    this.getMouseProjectionOnBall = function ( clientX, clientY ) {

        var mouseOnBall = new THREE.Vector3(
            ( clientX - _this.screen.width * 0.5 - _this.screen.offsetLeft ) / _this.radius,
            ( _this.screen.height * 0.5 + _this.screen.offsetTop - clientY ) / _this.radius,
            0.0
        );

        var length = mouseOnBall.length();

        if ( length > 1.0 ) {

            mouseOnBall.normalize();

        } else {

            mouseOnBall.z = Math.sqrt( 1.0 - length * length );

        }

        _eye.copy( _this.object.position ).subSelf( _this.target );

        var projection = _this.object.up.clone().setLength( mouseOnBall.y );
        projection.addSelf( _this.object.up.clone().crossSelf( _eye ).setLength( mouseOnBall.x ) );
        projection.addSelf( _eye.setLength( mouseOnBall.z ) );

        return projection;

    };

    this.rotateCamera = function () {

        var angle = Math.acos( _rotateStart.dot( _rotateEnd ) / _rotateStart.length() / _rotateEnd.length() );

        if ( angle ) {

            var axis = ( new THREE.Vector3() ).cross( _rotateStart, _rotateEnd ).normalize(),
                quaternion = new THREE.Quaternion();

            angle *= _this.rotateSpeed;

            quaternion.setFromAxisAngle( axis, -angle );

            quaternion.multiplyVector3( _eye );
            quaternion.multiplyVector3( _this.object.up );

            quaternion.multiplyVector3( _rotateEnd );

            if ( _this.staticMoving ) {

                _rotateStart.copy( _rotateEnd );

            } else {

                quaternion.setFromAxisAngle( axis, angle * ( _this.dynamicDampingFactor - 1.0 ) );
                quaternion.multiplyVector3( _rotateStart );

            }

        }

    };

    this.zoomCamera = function () {

        var factor = 1.0 + ( _zoomEnd.y - _zoomStart.y ) * _this.zoomSpeed;

        if ( factor !== 1.0 && factor > 0.0 ) {

            _eye.multiplyScalar( factor );

            if ( _this.staticMoving ) {

                _zoomStart.copy( _zoomEnd );

            } else {

                _zoomStart.y += ( _zoomEnd.y - _zoomStart.y ) * this.dynamicDampingFactor;

            }

        }

    };

    this.pinchToZoomCamera = function()
    {
        if (_startDistance == 0)
            return;

        var endDistance = _touchEnd[0].distanceTo(_touchEnd[1]);

        var scale = endDistance / _startDistance;

        var delta = (1-scale);

        var factor = 1 + delta * _this.zoomSpeed/3;
        if ( factor !== 1.0 && factor > 0.0 && factor < 1.5)
        {
            _eye.multiplyScalar(factor);
            _startDistance += ( endDistance - _startDistance ) * this.dynamicDampingFactor;
        }

    };

    this.panCamera = function () {

        var mouseChange = _panEnd.clone().subSelf( _panStart );

        if ( mouseChange.lengthSq() ) {

            mouseChange.multiplyScalar( _eye.length() * _this.panSpeed );

            var pan = _eye.clone().crossSelf( _this.object.up ).setLength( mouseChange.x );
            pan.addSelf( _this.object.up.clone().setLength( mouseChange.y ) );

            _this.object.position.addSelf( pan );
            _this.target.addSelf( pan );

            if ( _this.staticMoving ) {

                _panStart = _panEnd;

            } else {

                _panStart.addSelf( mouseChange.sub( _panEnd, _panStart ).multiplyScalar( _this.dynamicDampingFactor ) );

            }

        }

    };

    this.checkDistances = function () {

        if ( !_this.noZoom || !_this.noPan ) {

            if ( _this.object.position.lengthSq() > _this.maxDistance * _this.maxDistance ) {

                _this.object.position.setLength( _this.maxDistance );

            }

            if ( _eye.lengthSq() < _this.minDistance * _this.minDistance ) {

                _this.object.position.add( _this.target, _eye.setLength( _this.minDistance ) );

            }

        }

    };

    this.update = function () {

        _eye.copy( _this.object.position ).subSelf( _this.target );

        if ( !_this.noRotate ) {

            _this.rotateCamera();

        }

        if ( !_this.noZoom )
        {
            if (!_twoFingers)
                _this.zoomCamera();
            else
                _this.pinchToZoomCamera();
        }

        if ( !_this.noPan ) {

            _this.panCamera();

        }

        _this.object.position.add( _this.target, _eye );

        _this.checkDistances();

        _this.object.lookAt( _this.target );

        if ( lastPosition.distanceToSquared( _this.object.position ) > 0 ) {

            _this.dispatchEvent( changeEvent );

            lastPosition.copy( _this.object.position );

        }

    };

    // listeners

    function keydown( event ) {

        if ( ! _this.enabled ) return;

        //event.preventDefault();

        if ( _state !== STATE.NONE ) {

            return;

        } else if ( event.keyCode === _this.keys[ STATE.ROTATE ] && !_this.noRotate ) {

            _state = STATE.ROTATE;

        } else if ( event.keyCode === _this.keys[ STATE.ZOOM ] && !_this.noZoom ) {

            _state = STATE.ZOOM;

        } else if ( event.keyCode === _this.keys[ STATE.PAN ] && !_this.noPan ) {

            _state = STATE.PAN;

        }

        if ( _state !== STATE.NONE ) {

            _keyPressed = true;

        }

    }

    function keyup( event ) {

        if ( ! _this.enabled ) return;

        if ( _state !== STATE.NONE ) {

            _state = STATE.NONE;

        }

    }

    function mousedown( event ) {

        if ( ! _this.enabled ) return;

        event.preventDefault();
        event.stopPropagation();

        if ( _state === STATE.NONE ) {

            _state = event.button;

            if ( _state === STATE.ROTATE && !_this.noRotate ) {

                _rotateStart = _rotateEnd = _this.getMouseProjectionOnBall( event.clientX, event.clientY );

            } else if ( _state === STATE.ZOOM && !_this.noZoom ) {

                _zoomStart = _zoomEnd = _this.getMouseOnScreen( event.clientX, event.clientY );

            } else if ( !_this.noPan ) {

                _panStart = _panEnd = _this.getMouseOnScreen( event.clientX, event.clientY );

            }

        }

    }

    function mousemove( event ) {

        if ( ! _this.enabled ) return;

        if ( _keyPressed ) {

            _rotateStart = _rotateEnd = _this.getMouseProjectionOnBall( event.clientX, event.clientY );
            _zoomStart = _zoomEnd = _this.getMouseOnScreen( event.clientX, event.clientY );
            _panStart = _panEnd = _this.getMouseOnScreen( event.clientX, event.clientY );

            _keyPressed = false;

        }

        if ( _state === STATE.NONE ) {

            return;

        } else if ( _state === STATE.ROTATE && !_this.noRotate ) {

            _rotateEnd = _this.getMouseProjectionOnBall( event.clientX, event.clientY );

        } else if ( _state === STATE.ZOOM && !_this.noZoom ) {

            _zoomEnd = _this.getMouseOnScreen( event.clientX, event.clientY );

        } else if ( _state === STATE.PAN && !_this.noPan ) {

            _panEnd = _this.getMouseOnScreen( event.clientX, event.clientY );

        }

    }

    function mouseup( event ) {

        if ( ! _this.enabled ) return;

        event.preventDefault();
        event.stopPropagation();

        _state = STATE.NONE;

    }

    function mousewheel( event ) {

        if ( ! _this.enabled ) return;

        event.preventDefault();
        event.stopPropagation();

        var delta = 0;

        if ( event.wheelDelta ) { // WebKit / Opera / Explorer 9

            delta = event.wheelDelta / 40;

        } else if ( event.detail ) { // Firefox

            delta = - event.detail / 3;

        }

        _zoomStart.y += ( 1 / delta ) * 0.05;

    }



    // Calculating the euclidian distance between two fingers
    function calcFingerDistance( touch1, touch2 ) {
        var x = touch1.x - touch2.x;
        var y = touch1.y - touch2.y;

        return Math.sqrt( x * x + y * y );
    }

    function averageTouchLocation( touches ) {
        var touchX = 0, touchY = 0;

        for (var i = 0; i < touches.length; i++) {
            touchX += touches[i].clientX;
            touchY += touches[i].clientY;
        }

        return [touchX / touches.length, touchY / touches.length]
    }


    function touchstart( event ) {
        if ( event.touches.length === 1 ) {

            var touch = event.touches[0];
            event['clientX'] = touch.clientX;
            event['clientY'] = touch.clientY;
            event['button'] = 0;
            mousedown( event );

        }else if ( event.touches.length === 2) {
            _twoFingers = true;
            var touch1 = event.touches[0];
            var touch2 = event.touches[1];
            _state = STATE.PAN;

            var coords = averageTouchLocation( event.touches );

            _panStart = _panEnd = _this.getMouseOnScreen( coords[0], coords[1] );

            _initialTouchDist =  calcFingerDistance(_this.getMouseOnScreen(touch1.clientX, touch1.clientY), _this.getMouseOnScreen(touch2.clientX, touch2.clientY))

        }else {
            _twoFingers = false;
        }
    }

    function touchmove( event ) {
        if ( event.touches.length === 1 ){

            var touch = event.touches[0];
            event['clientX'] = touch.clientX;
            event['clientY'] = touch.clientY;
            mousemove( event );

        }else if( event.touches.length === 2) {
            _twoFingers = true;
            var touch1 = event.touches[0];
            var touch2 = event.touches[1];

            if(_state !== STATE.ZOOM) {
                _touchDist = calcFingerDistance(_this.getMouseOnScreen(touch1.clientX, touch1.clientY), _this.getMouseOnScreen(touch2.clientX, touch2.clientY))
                if((Math.abs(_initialTouchDist - _touchDist)) > _this.pinchZoomTriggerDist){
                    _state = STATE.ZOOM
                    _touchStart[0] = _this.getMouseOnScreen( touch1.clientX, touch1.clientY );
                    _touchStart[1] = _this.getMouseOnScreen( touch2.clientX, touch2.clientY );
                    _startDistance = _touchStart[0].distanceTo(_touchStart[1]);
                    _touchEnd = _touchStart;
                }else {
                    if ( _state === STATE.PAN && !_this.noPan ) {
                        var coords = averageTouchLocation( event.touches );

                        _panEnd = _this.getMouseOnScreen( coords[0], coords[1] );
                    }
                }
            }

            if ( _state === STATE.ZOOM && !_this.noZoom ) {
                _touchEnd[0] = _this.getMouseOnScreen( touch1.clientX, touch1.clientY );
                _touchEnd[1] = _this.getMouseOnScreen( touch2.clientX, touch2.clientY );

            }
        }else{
            _twoFingers = false;
        }
    }

    function touchend(event)
    {
        _twoFingers = false;
        _startDistance = 0;
        _initialTouchDist = _touchDist = 0
        mouseup( event );
    }


    this.domElement.addEventListener( 'contextmenu', function ( event ) { event.preventDefault(); }, false );

    this.domElement.addEventListener( 'mousemove', mousemove, false );
    this.domElement.addEventListener( 'mousedown', mousedown, false );
    this.domElement.addEventListener( 'mouseup', mouseup, false );

    this.domElement.addEventListener( 'touchmove', touchmove, false );
    this.domElement.addEventListener( 'touchstart', touchstart, false );
    this.domElement.addEventListener( 'touchend', touchend, false );

    this.domElement.addEventListener( 'DOMMouseScroll', mousewheel, false );
    this.domElement.addEventListener( 'mousewheel', mousewheel, false );

    window.addEventListener( 'keydown', keydown, false );
    window.addEventListener( 'keyup', keyup, false );

    this.handleResize();

};
