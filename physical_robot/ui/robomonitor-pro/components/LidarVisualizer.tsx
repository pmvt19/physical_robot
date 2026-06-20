
import React, { useRef, useEffect } from 'react';
import * as d3 from 'd3';
import { LIDAR_CONFIG } from '../constants';

interface LidarVisualizerProps {
  data: number[][]; // Array of [x, y] coordinates
}

const LidarVisualizer: React.FC<LidarVisualizerProps> = ({ data }) => {
  const svgRef = useRef<SVGSVGElement>(null);
  
  // References to D3 selections for specific layers
  const rootGRef = useRef<d3.Selection<SVGGElement, unknown, null, undefined> | null>(null);
  const gridGRef = useRef<d3.Selection<SVGGElement, unknown, null, undefined> | null>(null);
  const dotsGRef = useRef<d3.Selection<SVGGElement, unknown, null, undefined> | null>(null);
  const zoomRef = useRef<d3.ZoomBehavior<SVGSVGElement, unknown> | null>(null);

  // Constants for map projection
  // Defined here to be accessible by both Effects
  const WIDTH = 300;
  const HEIGHT = 300;
  const RADIUS = Math.min(WIDTH, HEIGHT) / 2 - 10;
  const PIXELS_PER_METER = RADIUS / LIDAR_CONFIG.maxRange; 

  // Helper: Update grid circles and labels based on zoom scale k
  const updateGrid = (k: number) => {
    if (!gridGRef.current) return;
    const g = gridGRef.current;

    // Calculate visible radius in meters at current zoom
    const visibleRadiusMeters = LIDAR_CONFIG.maxRange / k;
    
    // We want approximately 5 rings visible from center to edge
    const niceSteps = [0.1, 0.2, 0.25, 0.5, 1, 2, 5];
    
    // Find the step size that yields a ring count closest to 5
    let step = niceSteps[0];
    let minDiff = Infinity;
    
    for (const s of niceSteps) {
        const rings = visibleRadiusMeters / s;
        const diff = Math.abs(rings - 5);
        if (diff < minDiff) {
            minDiff = diff;
            step = s;
        }
    }

    // 2. Generate ticks based on step and max range
    const maxRange = LIDAR_CONFIG.maxRange;
    const ticks: number[] = [];
    // Start from step, go up to maxRange
    for (let r = step; r <= maxRange + 0.01; r += step) {
        ticks.push(r);
    }

    // 3. Data Join for Circles
    // Fix: Removed explicit generics from selectAll as it was causing "Untyped function calls" error
    const circles = g.selectAll(".grid-circle")
        .data(ticks, (d: any) => d); // Key by radius value

    circles.exit().remove();

    circles.enter()
        .append("circle")
        .attr("class", "grid-circle")
        .attr("fill", "none")
        .attr("stroke", "#334155")
        .attr("stroke-width", 1)
        .attr("vector-effect", "non-scaling-stroke")
        .style("opacity", 0.5)
        .merge(circles as any) // Update attributes for both new and existing
        .attr("r", (d: any) => d * PIXELS_PER_METER);

    // 4. Data Join for Labels
    // Fix: Removed explicit generics from selectAll
    const labels = g.selectAll(".grid-label")
        .data(ticks, (d: any) => d);

    labels.exit().remove();

    labels.enter()
        .append("text")
        .attr("class", "grid-label")
        .attr("fill", "#64748b")
        .merge(labels as any)
        .attr("y", (d: any) => -(d * PIXELS_PER_METER) + (10 / k))
        .attr("x", 2 / k)
        .text((d: any) => `${parseFloat(d.toFixed(2))}m`)
        .style("font-size", `${8 / k}px`); // Ensure font size scales inversely to zoom
  };

  // 1. Initialize SVG Structure and Zoom Behavior (Run Once)
  useEffect(() => {
    if (!svgRef.current) return;
    
    const svg = d3.select(svgRef.current);
    
    // Clear for hot-reload safety
    svg.selectAll("*").remove();

    // Create the container Group that will be transformed (panned/zoomed)
    const rootG = svg.append("g").attr("class", "root-content");
    rootGRef.current = rootG;

    // Layer 1: Grid (Bottom)
    gridGRef.current = rootG.append("g").attr("class", "grid-layer");

    // Layer 2: Robot Marker
    // Requirement: Robot is 210mm in diameter = 0.21m => Radius = 0.105m
    // This scales physically with the map (gets bigger as you zoom in)
    const robotRadiusPx = 0.105 * PIXELS_PER_METER;
    const robotLayer = rootG.append("g").attr("class", "robot-layer");
    
    robotLayer.append("circle")
      .attr("r", robotRadiusPx) 
      .attr("fill", "#0ea5e9");

    // FOV Indicator
    robotLayer.append("path")
      .attr("d", d3.arc()({
        innerRadius: 0,
        outerRadius: RADIUS,
        startAngle: Math.PI / 4,
        endAngle: -Math.PI / 4
      }) || "")
      .attr("fill", "#0ea5e9")
      .style("opacity", 0.05);

    // Layer 3: Lidar Points (Top)
    dotsGRef.current = rootG.append("g").attr("class", "dots-layer");

    // Define Zoom Behavior
    const zoom = d3.zoom<SVGSVGElement, unknown>()
        .scaleExtent([0.5, 20]) 
        .on("zoom", (event) => {
            const k = event.transform.k;
            rootG.attr("transform", event.transform);
            
            // Dynamic Updates
            updateGrid(k);
            
            // Keep dots constant visual size (inverse scale)
            if (dotsGRef.current) {
                dotsGRef.current.selectAll(".dot").attr("r", 2 / k);
            }
        });
    
    zoomRef.current = zoom;
    svg.call(zoom);

    // Initial Center View
    const initialTransform = d3.zoomIdentity.translate(WIDTH / 2, HEIGHT / 2).scale(1);
    svg.call(zoom.transform, initialTransform);

  }, []);

  // 2. Draw Data (Run on Data Change)
  useEffect(() => {
    if (!dotsGRef.current || !svgRef.current) return;
    
    // Get current zoom state
    const currentTransform = d3.zoomTransform(svgRef.current);
    const k = currentTransform.k;

    // Use d3 update pattern for points
    const pointsData = data.map((pt) => ({ 
        x: pt[0] * PIXELS_PER_METER, 
        y: -pt[1] * PIXELS_PER_METER 
    }));

    // Fix: Removed explicit generics from selectAll
    const dots = dotsGRef.current.selectAll(".dot")
      .data(pointsData);

    dots.exit().remove();

    dots.enter()
      .append("circle")
      .attr("class", "dot")
      .merge(dots as any)
      .attr("cx", (d: any) => d.x)
      .attr("cy", (d: any) => d.y)
      .attr("r", 2 / k) 
      .attr("fill", "#facc15")
      .style("opacity", 0.8)
      .attr("vector-effect", "non-scaling-stroke");

  }, [data]);

  const handleRecenter = () => {
    if (svgRef.current && zoomRef.current) {
        const svg = d3.select(svgRef.current);
        const transform = d3.zoomIdentity.translate(WIDTH / 2, HEIGHT / 2).scale(1);
        svg.transition().duration(750).call(zoomRef.current.transform, transform);
    }
  };

  return (
    <div className="flex flex-col items-center bg-slate-900 rounded-xl border border-slate-800 p-4 shadow-lg h-full overflow-hidden relative group">
      <div className="flex justify-between w-full mb-2 shrink-0 z-10">
          <h3 className="text-slate-400 text-xs font-bold uppercase tracking-wider">Lidar Scan (Interactive)</h3>
          <button 
            onClick={handleRecenter}
            className="text-[10px] bg-slate-800 hover:bg-slate-700 text-blue-400 px-2 py-0.5 rounded border border-slate-700 transition-colors"
          >
            Reset View
          </button>
      </div>
      
      <div className="flex-1 w-full min-h-0 relative bg-slate-950/50 rounded border border-slate-800/50 overflow-hidden">
         <svg 
            ref={svgRef} 
            width="100%" 
            height="100%" 
            viewBox="0 0 300 300" 
            className="absolute inset-0 w-full h-full block cursor-crosshair touch-none" 
            preserveAspectRatio="xMidYMid meet" 
         />
         <div className="absolute bottom-2 right-2 text-[10px] text-slate-600 pointer-events-none opacity-0 group-hover:opacity-100 transition-opacity">
            Scroll to Zoom • Drag to Pan
         </div>
      </div>
    </div>
  );
};

export default LidarVisualizer;
