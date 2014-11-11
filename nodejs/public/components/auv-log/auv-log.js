Polymer({
    ready: function() {
        this.initDataFull();
    },
    domReady: function() {
        var list = this.$.listDiv;
        list.scrollTop = list.scrollHeight;
    },
    initData: function() {
        this.data = [];
    },
    initDataFull: function() {
        this.data = this.generateData();
    },
    deleteData: function() {
        this.data = null;
    },
    emptyData: function() {
        this.data.splice(0, this.data.length);
    },
    generateData: function() {
        var data = [];
        for (var i = 0; i < 20; i++) {
            data.push({
                id: i,
                str: 'This id ' + i,
                level: (function(i) {
                    if (i == 0) {
                        return 'warn';
                    } else if (i % 9 == 0) {
                        return 'error';
                    } else if (i % 3 == 0) {
                        return 'debug';
                    } else {
                        return 'log';
                    }
                })(i)
            });
        }
        return data;
    }
});
