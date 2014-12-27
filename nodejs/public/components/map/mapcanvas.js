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
        dataPoints:dps 
			}];
		//});
//pushing the data into the chart
    /*chart.options.data = [];
    chart.options.data.push(object1);
    object1.dataPoints = [
        { x: 51, y: 10,name:"initial" }*/
        /*{ x: 42, y: 20,name:"10ms" },
        { x: 30, y: 55,name:"20ms" },
        { x: 12, y: -10,name:"30ms" },
		{ x: 29, y: -50,name:"40ms" },
		{ x: -20, y: 98,name:"50ms" },
		{ x: 16, y: -30,name:"60ms" },
		{ x: 9, y: 0,name:"70ms" },*/
       
        //];

    //rendering the chart when the page loads 
        chart.render();

    var newX;
    var newY;
    //after window load
  
    //Math.floor((Math.random() * 100) + 1) returns a number between 1 and 100
    //create a function that spews out numbers every second
    function timedInput() {
        window.setInterval(function(){
            newX = Math.floor((Math.random() * 100) + 1);
            console.log("newX is " + newX);
            newY = Math.floor((Math.random() * 100) + 1);
            console.log("newY is " + newY);
        },1000);
    }
    timedInput();

    //set the graph to "check" the value and change it after 5 seconds 
    setTimeout(function(){
       
        dps[0].push({
        x:50,
        y:50,
        name:"change"
    });
       /* dps[0].x=Math.floor((Math.random() * 100) + 1);
        dps[0].y=Math.floor((Math.random() * 100) + 1);
        dps[0].name= "changed";*/
        
        chart.render;
        console.log("It worked!");
        console.log(dps[0].x);
        console.log(dps[0].y);
        console.log(dps[0].name);
    },5000);
    
}