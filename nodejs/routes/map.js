//Route for Map Polymer element
//Nov 22, 2014

var express = require('express');
var router = express.Router();

/* GET Map's Page. */
router.get('/', function(req, res) {
    res.render('map', {
        title: 'McGill Robotics Map'
        
    });
});

module.exports = router;
