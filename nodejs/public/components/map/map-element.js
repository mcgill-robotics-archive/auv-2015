Polymer({
    ready: function(){
        //random function
        this.$.el.textContent = this.owner + " is ready!";
    },
    owner: "Henry",
    color: "red"
});