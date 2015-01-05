//Map Element JS by Henry Yu
//December 2014 
//Working Progress, to be eventually moved inside a polymer element

window.onload = function () {

//creating the chart container
    var chart = new CanvasJS.Chart("chartContainer");
    
//constructing the title
    chart.options.title = {
            text: "Object Distance from Robot",      
            fontFamily: "arial black",
            fontColor: "DarkSlateGrey"};
    
//constructing the X axis 
    chart.options.axisX = {
            title:"Horizontal Position",
            titleFontFamily: "arial",
            titleFontSize:12,
            minimum:-100,
            maximum:100,
            interval:20,
            gridThickness:0.8};
    
//constructing the Y axis
    chart.options.axisY = {         
		title: "Vertical Position",
        titleFontFamily: "arial",
        titleFontSize: 12,
        minimum:-100,
        maximum:100,
        interval:20,
        gridThickness:0.8};
    
//creating the first object 
    /*var object1 = {
        type: "scatter",  
        toolTipContent: "<span style='\"'color: {color};'\"'><strong>{name}</strong></span> <br/> <strong>Y-Position</strong> {y} <br/> <strong>X-Position</strong> {x} ",
        name: "Object 1",
        showInLegend: true
    };*/
    
//data points 
    var dps = [{
        x:30,
        y:40,
        name:"initial"
    }]; 
    chart.options.data = [{
        type: "scatter",
        toolTipContent: "<span style='\"'color: {color};'\"'><strong>{name}</strong></span> <br/> <strong>Y-Position</strong> {y} <br/> <strong>X-Position</strong> {x} ",
        name: "Object 1",
        showInLegend: true,
        dataPoints:dps 
    }];
		//});

    //rendering the chart when the page loads 
    chart.render();

    var xVal;
    var yVal;
    var updateInterval = 1000;
    
    var updateChart = function () {
        xVal = Math.floor((Math.random() * 100) + 1);
        yVal = Math.floor((Math.random() * 100) + 1);
        dps.push({
            x: xVal,
            y: yVal,
            name: "changed"
        });
        console.log(xVal);
        console.log(yVal);
        chart.render();
    };
    
    setInterval(function(){updateChart()}, updateInterval);
    
/*    function timedInput() {
        window.setInterval(function(){
            newX = Math.floor((Math.random() * 100) + 1);
            console.log("newX is " + newX);
            newY = Math.floor((Math.random() * 100) + 1);
            console.log("newY is " + newY);
        },1000);
    }
    timedInput();*/

    //set the graph to "check" the value and change it after 5 seconds 
/*    setTimeout(function(){
       
        dps[0].push({
        x:50,
        y:50,
        name:"change"
    });
        dps[0].x=Math.floor((Math.random() * 100) + 1);
        dps[0].y=Math.floor((Math.random() * 100) + 1);
        dps[0].name= "changed";
        
        chart.render;
        console.log("It worked!");
        console.log(dps[0].x);
        console.log(dps[0].y);
        console.log(dps[0].name);
    },5000);*/
    
}