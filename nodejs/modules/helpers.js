var fs = require('fs');
var path = require('path');

function isArray(obj) {
    return Object.prototype.toString.call(obj) === '[object Array]';
}

function isExt(ext, file) {
    if (path.extname(file) === '.' + ext) {
        return true;
    }
    return false;
}

function filterExts(exts, file) {
    if (isArray(exts)) {
        return exts.some(function(ext) {
            return (isExt(ext, file));
        });
    }
    return (path.extname(file) === '.' + exts);
}

module.exports = {
    isArray: isArray,
    isExt: isExt,
    filterExts: filterExts
};
