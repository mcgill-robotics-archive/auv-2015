Polymer('depth-graph', {
    $(function () {
        $(document).ready(function () {
            Highcharts.setOptions({
                global:{
                    useUTC: false
                }
            });

            $('container').highcharts({
                chart:{
                    type: 'spline', //conect points with a curve
                    animation: Highcharts.svg,
                    marginRight: 10,
                    events:{


                        //load new values to the chart at an interval of time
                        load: function () {
                            var series = this.series[0];//an array with x and y values
                            setInterval(function(){
                                var x =(new Date()).getTime(), //current time
                                    y = Math.random(); //generate random number for y value
                                series.addPoint([x,y], true,true); //add point to the
                            }, 1000);//update chart every 1 second


                        }
                    }
                },

                //setup of the graph
                title: {
                    text:' '
                },
                xAxis:{
                    type: 'time',
                    tickPixelInterval:150
                },
                yAxis:{
                    title:{
                        text:'value'
                    },
                    plotlines:[{
                        value:0,
                        width: 1,
                        color: '#808080'
                    }]

                },
                tooltip:{
                    enabled:false
                },

                legend:{
                    enabled:false
                },
                exporting:{
                    enabled:false
                },
                series:[{
                    name: 'data',
                    data:(function(){
                        //Input of initial data (20 pts)
                        var data=[],
                            time=(new Date()).getTime(),
                            i;
                        for (i=-19; i<=0;i++){
                            data.push({
                                x:time+i*1000,
                                y:Math.random()
                            });
                        }
                        return data;
                    }())
                }]
            });
        });
    });
});
