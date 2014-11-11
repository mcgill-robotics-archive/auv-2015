var express = require('express');
var router = express.Router();

module.exports = function(development) {
    /* GET home page. */
    router.get('/', function(req, res) {
        res.render('index', {
            title: 'Welcome',
            dev: development
        });
    });

    return router;
};
