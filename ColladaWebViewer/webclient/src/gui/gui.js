/*!
 * jQuery UI Touch Punch 0.2.2
 *
 * Copyright 2011, Dave Furfero
 * Dual licensed under the MIT or GPL Version 2 licenses.
 *
 * Depends:
 *  jquery.ui.widget.js
 *  jquery.ui.mouse.js
 */
(function ($) {

    // Detect touch support
    $.support.touch = 'ontouchend' in document;

    // Ignore browsers without touch support
    if (!$.support.touch) {
        return;
    }

    var mouseProto = $.ui.mouse.prototype,
        _mouseInit = mouseProto._mouseInit,
        touchHandled;

    /**
     * Simulate a mouse event based on a corresponding touch event
     * @param {Object} event A touch event
     * @param {String} simulatedType The corresponding mouse event
     */
    function simulateMouseEvent (event, simulatedType) {

        // Ignore multi-touch events
        if (event.originalEvent.touches.length > 1) {
            return;
        }

        event.preventDefault();

        var touch = event.originalEvent.changedTouches[0],
            simulatedEvent = document.createEvent('MouseEvents');

        // Initialize the simulated mouse event using the touch event's coordinates
        simulatedEvent.initMouseEvent(
            simulatedType,    // type
            true,             // bubbles
            true,             // cancelable
            window,           // view
            1,                // detail
            touch.screenX,    // screenX
            touch.screenY,    // screenY
            touch.clientX,    // clientX
            touch.clientY,    // clientY
            false,            // ctrlKey
            false,            // altKey
            false,            // shiftKey
            false,            // metaKey
            0,                // button
            null              // relatedTarget
        );

        // Dispatch the simulated event to the target element
        event.target.dispatchEvent(simulatedEvent);
    }

    /**
     * Handle the jQuery UI widget's touchstart events
     * @param {Object} event The widget element's touchstart event
     */
    mouseProto._touchStart = function (event) {

        var self = this;

        // Ignore the event if another widget is already being handled
        if (touchHandled || !self._mouseCapture(event.originalEvent.changedTouches[0])) {
            return;
        }

        // Set the flag to prevent other widgets from inheriting the touch event
        touchHandled = true;

        // Track movement to determine if interaction was a click
        self._touchMoved = false;

        // Simulate the mouseover event
        simulateMouseEvent(event, 'mouseover');

        // Simulate the mousemove event
        simulateMouseEvent(event, 'mousemove');

        // Simulate the mousedown event
        simulateMouseEvent(event, 'mousedown');
    };

    /**
     * Handle the jQuery UI widget's touchmove events
     * @param {Object} event The document's touchmove event
     */
    mouseProto._touchMove = function (event) {

        // Ignore event if not handled
        if (!touchHandled) {
            return;
        }

        // Interaction was not a click
        this._touchMoved = true;

        // Simulate the mousemove event
        simulateMouseEvent(event, 'mousemove');
    };

    /**
     * Handle the jQuery UI widget's touchend events
     * @param {Object} event The document's touchend event
     */
    mouseProto._touchEnd = function (event) {

        // Ignore event if not handled
        if (!touchHandled) {
            return;
        }

        // Simulate the mouseup event
        simulateMouseEvent(event, 'mouseup');

        // Simulate the mouseout event
        simulateMouseEvent(event, 'mouseout');

        // If the touch interaction did not move, it should trigger a click
        if (!this._touchMoved) {

            // Simulate the click event
            simulateMouseEvent(event, 'click');
        }

        // Unset the flag to allow other widgets to inherit the touch event
        touchHandled = false;
    };

    /**
     * A duck punch of the $.ui.mouse _mouseInit method to support touch events.
     * This method extends the widget with bound touch event handlers that
     * translate touch events to mouse events and pass them to the widget's
     * original mouse event handling methods.
     */
    mouseProto._mouseInit = function () {

        var self = this;

        // Delegate the touch handlers to the widget's element
        self.element
            .bind('touchstart', $.proxy(self, '_touchStart'))
            .bind('touchmove', $.proxy(self, '_touchMove'))
            .bind('touchend', $.proxy(self, '_touchEnd'));

        // Call the original $.ui.mouse init method
        _mouseInit.call(self);
    };

})(jQuery);


/*
 Function for centering any element horizontally or/and vertically

 */
(function( $ ){
    $.fn.extend({
        center: function (options) {
            var settings =  $.extend({
                inside: window, // In what context to center
                transition: 0, // Transition time, milliseconds
                minX: 0, // min left element value, px
                minY: 0, // min top element value, px
                withScrolling: true, // handle scroll bar
                vertical: true, // center vertically
                horizontal: true // center horizontally
            }, options);

            return this.each(function() {
                var props = {position:'absolute'};
                if (settings.vertical) {
                    var top = ($(settings.inside).height() - $(this).outerHeight()) / 2;
                    if (settings.withScrolling) top += $(settings.inside).scrollTop() || 0;
                    top = (top > settings.minY ? top : settings.minY);
                    $.extend(props, {top: top+'px'});
                }
                if (settings.horizontal) {
                    var left = ($(settings.inside).width() - $(this).outerWidth()) / 2;
                    if (settings.withScrolling) left += $(settings.inside).scrollLeft() || 0;
                    left = (left > settings.minX ? left : settings.minX);
                    $.extend(props, {left: left+'px'});
                }
                if (settings.transition > 0) $(this).animate(props, settings.transition);
                else $(this).css(props);
                return $(this);
            });
        }
    });
})( jQuery );

/*
 The main GUI init function

 */

(function($) {

    $(document).ready(function(){


        // *** Initilizing the left menu ***

        var _leftMenu = $("#leftMenu");
        _leftMenu.css('left', -_leftMenu.outerWidth())
        _leftMenu.center({horizontal: false, inside: $("#content")});
        $(window).bind('resize', function() {
            $("#leftMenu").center({transition:300, horizontal: false, inside: $("#content")});
        });


        // Adding accordion-style menu
        $( "#accordion" ).accordion({
            fillSpace: true
        })

        _leftMenu.triggerBar = $("#trigger1")
        _leftMenu.triggerBar.position({
            of: _leftMenu,
            at: "right center",
            my: "right center",
            offset: "22px 0"
        })

        var iconArea = $("<div/>", {
            class: "ui-state-blank",
            css: {height: "20px"}
        }).appendTo(_leftMenu.triggerBar)
        iconArea.position({
            of: _leftMenu.triggerBar,
            my: "center",
            at: "center"
        })

        var icon = _leftMenu.triggerBar.arrow = $("<div/>", {
            class: "ui-icon ui-icon-triangle-1-e"
        }).appendTo(iconArea)
        icon.position({
            of: iconArea,
            my: "center",
            at: "center"
        })

        _leftMenu.triggerBar.bind('click',function(){
            var parent = $(this).parent()
            parent.animate({
                left: parseInt(parent.css('left'),10) == 0 ?
                    - parent.outerWidth() :
                    0
            });
            if (!_leftMenu.attr("isOpened")){
                _leftMenu.triggerBar.arrow[0].className = "ui-icon ui-icon-triangle-1-w"
                _leftMenu.attr("isOpened",true);
            }else{
                _leftMenu.triggerBar.arrow[0].className = "ui-icon ui-icon-triangle-1-e"
                _leftMenu.removeAttr("isOpened");
            }
        })


        // Initializing sliders in the left menu
        var initSlider = function(options, el, resultEl){
            var slider = $(el).slider({
                range: "max",
                min: options.min,
                max: options.max,
                step: options.step,
                value: 1,
                slide: function( event, ui ) {
                    $( resultEl ).text( ui.value )
                    $(this).trigger('changed', ui.value)
                }
            }).css({width:"100%"})

            $(resultEl).text( $(slider).slider( "value" ))

            return slider
        }

        var _scale = {}
        _scale.scaleX = initSlider({min: 1, max: 5, step: 0.5}, "#scaleX", "#amount1")

        _scale.scaleY = initSlider({min: 1, max: 5, step: 0.5}, "#scaleY", "#amount2")

        _scale.scaleZ = initSlider({min: 1, max: 5, step: 0.5}, "#scaleZ", "#amount3")


        var _sceneParams = {}

        _sceneParams.lightIntensity = initSlider({min: 0, max: 5, step: 0.1}, "#light", "#intensity")

        // *** Initializing right menu ***

        var _rightMenu = $("#rightMenu");
        _rightMenu.css('right', -_rightMenu.outerWidth())
        _rightMenu.center({horizontal: false, inside: $("#content")});
        $(window).bind('resize', function() {
            $("#rightMenu").center({transition:300, horizontal: false, inside: $("#content")});
        });


        _rightMenu.triggerBar = $("#trigger2")
        _rightMenu.triggerBar.position({
            of: _rightMenu,
            at: "left center",
            my: "left center",
            offset: "-22px 0"
        })

        _rightMenu.open = false

        iconArea = $("<div/>", {
            class: "ui-state-blank",
            css: {height: "20px"}
        }).appendTo(_rightMenu.triggerBar)
        iconArea.position({
            of: _rightMenu.triggerBar,
            my: "center",
            at: "center"
        })

        icon = _rightMenu.triggerBar.arrow = $("<div/>", {
            class: "ui-icon ui-icon-triangle-1-w"
        }).appendTo(iconArea)
        icon.position({
            of: iconArea,
            my: "center",
            at: "center"
        })

        _rightMenu.triggerBar.bind('click',function(){
            var parent = $(this).parent()
            parent.animate({
                right: parseInt(parent.css('right'),10) == 0 ?
                    -parent.outerWidth():
                    0
            })

            if (!_rightMenu.attr("isOpened")){
                _rightMenu.triggerBar.arrow[0].className = "ui-icon ui-icon-triangle-1-e"
                _rightMenu.attr("isOpened",true);
            }else{
                _rightMenu.triggerBar.arrow[0].className = "ui-icon ui-icon-triangle-1-w"
                _rightMenu.removeAttr("isOpened");
            }

        })


        // Adding file listing into the right menu

        var _fileList =  $("#fileList")

        _fileList.addItem = function(name) {
            var self = this
            $(this).append($("<li class='ui-state-default' value='"+name+"'/>").text(name)
                .click(function() {
                    $(this).addClass("ui-selected").siblings().removeClass("ui-selected");

                    if(typeof(self.callback) === 'function')
                        self.callback(name)
                }))
        }

        _fileList.setCallback = function(callback) {
            this.callback = callback
            return this
        }


        _fileList.getItem = function(value) {
            return this.find("li[value='"+value+"']")
        }

        _fileList.toggleLoaded = function(value) {
            var item = this.getItem(value)
            if(item)
                item.toggleClass("loaded")
        }


        _fileList.removeItem = function(name) {
            $(this).remove("li[value='"+name+"']")
        }


        // *** Saving GUI configuration ***

        $.gui = {
            'objectScale' : _scale,
            'sceneParams' : _sceneParams,
            'fileList' : _fileList
        }
    })

})( jQuery );
