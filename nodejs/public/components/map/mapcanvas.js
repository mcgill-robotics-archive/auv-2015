//this is a testing chart

window.onload = function () {
		var chart = new CanvasJS.Chart("chartContainer",
		{
            title:{
				text: "Object Distance from Robot",      
				fontFamily: "arial black",
				fontColor: "DarkSlateGrey"
            },
			axisX: {
				title:"Horizontal Position",
				titleFontFamily: "arial",
                titleFontSize:12,
                minimum:-100,
                maximum:100,
                interval:20,
                gridThickness:0.8

			},
            axisY:{
				title: "Vertical Position",
				titleFontFamily: "arial",
				titleFontSize: 12,
                minimum:-100,
                maximum:100,
                interval:20,
                gridThickness:0.8
			},
            data: [
			{        
				type: "scatter",  
				toolTipContent: "<span style='\"'color: {color};'\"'><strong>{name}</strong></span> <br/> <strong>Y-Position</strong> {y} <br/> <strong>X-Position</strong> {x} ",
				dataPoints: [

				
				{ x: 51, y: 10,name:"0ms" },
				{ x: 42, y: 20,name:"10ms" },
				{ x: 30, y: 55,name:"20ms" },
				{ x: 12, y: -10,name:"30ms" },
				{ x: 29, y: -50,name:"40ms" },
				{ x: -20, y: 98,name:"50ms" },
				{ x: 16, y: -30,name:"60ms" },
				{ x: 9, y: 0,name:"70ms" },
                { x: -80, y: 0,name:"80ms" }

				]
			}
			]
            
            
            });

chart.render();
}