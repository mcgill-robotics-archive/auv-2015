(function() {
    var values = {};
    Polymer({
        ready: function() {
            this.values = values;
            for (var i = 0; i < this.attributes.length; ++i) {
                var attr = this.attributes[i];
                values[attr.nodeName] = attr.value;
            }
        }
    });
})();
