Polymer('aimee-card', {

    ready: function(){
        this.dataset =[];
        this.values.forEach(function(val,i){
            this.dataset.push({
                //fillColor: "rgba("+this.color[i]+",0.5)",
                strokeColor: "rgba("+this.color[i]+",1)",
                pointColor: "rgba("+this.color[i]+",1)",
                data: this.values[i]
            });
        },this);

        this.data= {labels: this.labels, dataset: this.datasets};
        console.log(this.data);

        this.ctx=this.$.canvas.getContext('2d');
        this.chart= new Chart(this.ctx).Line(this.data);
    },


    labels:["a", "b", "c"],
    values:[1,2,3],
    colors:[],

});
