Polymer({
    // polymer element prototype goes here
    publish: { // publically avaialble info for this element (public api)
        // public properties
    },
    ready: function() {
        // executed when polymer is ready
    },
    exampleProperty: 'exampleValue2', // element scope: properties and functions
    get prop() { // example es5 getter; to use it just write: example.prop
        return this.exampleProperty;
    },
    foo: function() {
        // example function
        this.super(); //call parent foo()
    },
    //see documentation for more
    //other lifecycle methods
    created: function() {
        // something
    },
    //    ready: function() {...
    //    },
    attached: function() {
        // something
    },
    domReady: function() {
        // something
    },
    detached: function() {
        // something
    },
    attributeChanged: function(attrName, oldVal, newVal) {
        //var newVal = this.getAttribute(attrName);
        console.log(attrName, 'old: ' + oldVal, 'new:', newVal);
    },
});
