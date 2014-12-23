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
    var object1 = {
        type: "scatter",  
        toolTipContent: "<span style='\"'color: {color};'\"'><strong>{name}</strong></span> <br/> <strong>Y-Position</strong> {y} <br/> <strong>X-Position</strong> {x} ",
        name: "Object 1",
        showInLegend: true
    };
//pushing the data into the chart
    chart.options.data = [];
    chart.options.data.push(object1);
    object1.dataPoints = [
        { x: 51, y: 10,name:"0ms" },
        { x: 42, y: 20,name:"10ms" },
        { x: 30, y: 55,name:"20ms" },
        { x: 12, y: -10,name:"30ms" },
		{ x: 29, y: -50,name:"40ms" },
		{ x: -20, y: 98,name:"50ms" },
		{ x: 16, y: -30,name:"60ms" },
		{ x: 9, y: 0,name:"70ms" },
        { x: -80, y: 0,name:"80ms" }
        ];

    var xvalue = document.getElementById("xvalue");
    var yvalue = document.getElementById("yvalue");
    
//rendering the chart
    chart.render();
}