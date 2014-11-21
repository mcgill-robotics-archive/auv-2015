//Route code for Henry's test web-page 
//Nov 21, 2014

var express = require('express');
var router = express.Router();

/* GET Henry's Page. */
router.get('/', function(req, res) {
    res.render('henry', {
        title: 'Henry\'s test page'
        
    });
});

module.exports = router;
