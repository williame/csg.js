// Constructive Solid Geometry (CSG2) is a modeling technique that uses Boolean
// operations like union and intersection to combine 3D solids. This library
// implements CSG2 operations on meshes elegantly and concisely using BSP trees,
// and is meant to serve as an easily understandable implementation of the
// algorithm. All edge cases involving overlapping coplanar polygons in both
// solids are correctly handled.
// 
// Example usage:
// 
//		 var cube = CSG2.cube();
//		 var sphere = CSG2.sphere({ radius: 1.3 });
//		 var polygons = cube.subtract(sphere).toPolygons();
// 
// ## Implementation Details
// 
// All CSG2 operations are implemented in terms of two functions, `clipTo()` and
// `invert()`, which remove parts of a BSP tree inside another BSP tree and swap
// solid and empty space, respectively. To find the union of `a` and `b`, we
// want to remove everything in `a` inside `b` and everything in `b` inside `a`,
// then combine polygons from `a` and `b` into one solid:
// 
//		 a.clipTo(b);
//		 b.clipTo(a);
//		 a.build(b.allPolygons());
// 
// The only tricky part is handling overlapping coplanar polygons in both trees.
// The code above keeps both copies, but we need to keep them in one tree and
// remove them in the other tree. To remove them from `b` we can clip the
// inverse of `b` against `a`. The code for union now looks like this:
// 
//		 a.clipTo(b);
//		 b.clipTo(a);
//		 b.invert();
//		 b.clipTo(a);
//		 b.invert();
//		 a.build(b.allPolygons());
// 
// Subtraction and intersection naturally follow from set operations. If
// union is `A | B`, subtraction is `A - B = ~(~A | B)` and intersection is
// `A & B = ~(~A | ~B)` where `~` is the complement operator.
// 
// ## License
// 
// Copyright (c) 2011 Evan Wallace (http://madebyevan.com/), under the MIT license.

// # class CSG2

// Holds a binary space partition tree representing a 3D solid. Two solids can
// be combined using the `union()`, `subtract()`, and `intersect()` methods.

var CSG2 = function() {
	this.polygons = [];
};

// Construct a CSG2 solid from a list of `CSG2.Polygon` instances.
CSG2.fromPolygons = function(polygons) {
	var csg = new CSG2();
	csg.polygons = polygons;
	return csg;
};

CSG2.getBounds = function(polygons) {
	if(!polygons || !polygons.length)
		return { min: CSG2.Vertex(0,0,0), max: CSG2.Vertex(0,0,0), };
	var pos = polygons[0].vertices[0].pos;
	var min = pos.clone(), max = pos.clone();
	for(var i=1; i<polygons.length; i++) {
		var vertices = polygons[i].vertices;
		for(var j in vertices) {
			pos = vertices[j].pos;
			min.x = Math.min(min.x,pos.x);
			min.y = Math.min(min.y,pos.y);
			min.z = Math.min(min.z,pos.z);
			max.x = Math.max(max.x,pos.x);
			max.y = Math.max(max.y,pos.y);
			max.z = Math.max(max.z,pos.z);
		}
	}
	return { min: min, max: max };
}

CSG2.prototype = {
	clone: function() {
		var csg = new CSG2();
		if(this.polygons) csg.polygons = this.polygons.map(function(p) { return p.clone(); });
		return csg;
	},

	toPolygons: function() {
		return this.polygons;
	},
	
	scale: function(s) {
		var polygons = this.toPolygons().map(function(p) {
				var vertices = p.vertices.map(function(v) { return new CSG2.Vertex(v.pos.scale(s),v.normal); });
				return new CSG2.Polygon(vertices,p.shared);
		});
		return CSG2.fromPolygons(polygons);
	},

	translate: function(t) {
		var polygons = this.toPolygons().map(function(p) {
				var vertices = p.vertices.map(function(v) { return new CSG2.Vertex(v.pos.translate(t),v.normal); });
				return new CSG2.Polygon(vertices,p.shared);
		});
		return CSG2.fromPolygons(polygons);
	},
	
	rotate: function(theta,axis_start,axis_stop) {
	  theta = Math.min(Math.max(0, theta), 360) * (Math.PI/180);
		var polygons = this.toPolygons().map(function(p) {
				var vertices = p.vertices.map(function(v) {
				    return new CSG2.Vertex(v.pos.rotate(theta,axis_start,axis_stop),v.normal.rotate(theta,axis_start,axis_stop));
				  });
				return new CSG2.Polygon(vertices,p.shared);
		});
		return CSG2.fromPolygons(polygons);
	},
	
	join: function(csg) {
	  return CSG2.fromPolygons(this.toPolygons().concat(csg.toPolygons()));
	},
	
	tidy: function() {
	  //### work in progress, unfinished
	  // Hertel-Mehlhorn
	  // turn our polygons into vertex indices
	  var polygons = this.toPolygons(), vertices = [], neighbours = [], index = {};
		for(var p=0; p<polygons.length; p++) {
		  var polygon = polygons[p];
		  polygon.joined = null;
		  polygon.idx = p;
		  polygon.indices = [];
			for(var v in polygon.vertices) {
			  v = polygon.vertices[v];
			  if(!(v in index)) {
			    index[v] = vertices.length;
			    polygon.indices.push(vertices.length);
			    vertices.push(v);
			    neighbours.push([p]);
			  } else {
			    polygon.indices.push(index[v]);
			    neighbours[index[v]].push(p);
			  }
			}
		}
		for(var neighbour in neighbours)
		  neighbours[neighbour].sort();
		var find_opposite = function(a,b,p) {
      // find the neighbour of a and b that is not polygon p
      var na = neighbours[a], nb = neighbours[b];
      for(var i=0, j=0; i<na.length && j<nb.length; ) {
        var ii = na[i], jj = nb[j], cmp = ii-jj;
        if(cmp == 0 && ii != p)
          return polygons[ii];
        if(cmp >= 0)
          j++;
        if(cmp <= 0)
          i++;
      }
      return null;
		};
		var wrap = function(x,length) { return x==length? 0: x<0? length-1: x; };
		var calc_winding = function(a,b,c) {
		  b = vertices[b];
		  var ba = b.minus(vertices[a]).unit();
		  var bc = b.minus(vertices[c]).unit();
		  var theta = Math.acos(ba.dot(bc));
		  return theta < Math.PI;
		};
		var joins = 0;
		for(var p in polygons) {
		  var polygon = polygons[p];
		  if(polygon.joined) continue;
		  for(var i=0; i<polygon.indices.length; i++) {
		    var j = wrap(i+1,polygon.indices.length);
		    var opp = find_opposite(polygon.indices[i],polygon.indices[j],polygon.idx);
		    if(opp === null) continue;
		    while(opp.joined) opp = opp.joined; // follow the tombstones
		    if(opp === polygon) continue; // its already joined to us
		    if(opp.shared !== polygon.shared) continue; // incompatible?
		    if(Math.abs(1-opp.plane.normal.dot(polygon.plane.normal)) > CSG2.Plane.EPSILON) continue; // different normal?
		    //### does joining with opp keep us convex?
		    var a = opp.indices.indexOf(polygon.indices[i]);
		    var b = opp.indices.indexOf(polygon.indices[j]);
		    if(a > b) { var temp = a; a = b; b = temp; }
		    var prev = wrap(i-1,polygon.indices.length);
		    var opp_prev = wrap(a-1,opp.indices.length);
		    //### join them
		    polygon.vertices.splice(i,1);
		    polygon.vertices = polygon.vertices.concat(opp.vertices);
		    polygon.indices = polygon.indices.concat(opp.indices);
		    opp.joined = polygon;
		    joins++;
		  }
		}
		// remove joined
		return CSG2.fromPolygons(polygons.filter(function(polygon) { return !polygon.joined; }));
	},
	
	trivia: function() {
	  var start = new Date().getTime();
	  var counts = function(csg) {
      var lengths = {}, polygons = csg.toPolygons(), triangles = 0;
      for(var p in polygons) {
        var length = polygons[p].vertices.length;
        lengths[length] = (lengths[length] || 0) + 1;
        triangles += length-2;
      }
      var s = "";
      for(var l in lengths) {
        if(s.length) s += ",";
        s += l+"="+lengths[l];
      }
      return ""+triangles+" triangles; "+s;
    }
    return counts(this); //+" -> "+counts(this.tidy())+" ("+(new Date().getTime()-start)+"ms)";
	},
	
	// Return a new CSG2 solid representing space in either this solid or in the
	// solid `csg`. Neither this solid nor the solid `csg` are modified.
	// 
	//		 A.union(B)
	// 
	//		 +-------+						+-------+
	//		 |			 |						|				|
	//		 |	 A	 |						|				|
	//		 |		+--+----+		=		|				+----+
	//		 +----+--+		|				+----+			 |
	//					|		B		|						 |			 |
	//					|				|						 |			 |
	//					+-------+						 +-------+
	// 
	union: function(csg) {
		var a = new CSG2.Node(this.clone().toPolygons());
		var b = new CSG2.Node(csg.clone().toPolygons());
		a.clipTo(b);
		b.clipTo(a);
		b.invert();
		b.clipTo(a);
		b.invert();
		a.build(b.allPolygons());
		return CSG2.fromPolygons(a.allPolygons());
	},

	// Return a new CSG2 solid representing space in this solid but not in the
	// solid `csg`. Neither this solid nor the solid `csg` are modified.
	// 
	//		 A.subtract(B)
	// 
	//		 +-------+						+-------+
	//		 |			 |						|				|
	//		 |	 A	 |						|				|
	//		 |		+--+----+		=		|		 +--+
	//		 +----+--+		|				+----+
	//					|		B		|
	//					|				|
	//					+-------+
	// 
	subtract: function(csg) {
		var a = new CSG2.Node(this.clone().toPolygons());
		var b = new CSG2.Node(csg.clone().toPolygons());
		a.invert();
		a.clipTo(b);
		b.clipTo(a);
		b.invert();
		b.clipTo(a);
		b.invert();
		a.build(b.allPolygons());
		a.invert();
		return CSG2.fromPolygons(a.allPolygons());
	},

	// Return a new CSG2 solid representing space both this solid and in the
	// solid `csg`. Neither this solid nor the solid `csg` are modified.
	// 
	//		 A.intersect(B)
	// 
	//		 +-------+
	//		 |			 |
	//		 |	 A	 |
	//		 |		+--+----+		=		+--+
	//		 +----+--+		|				+--+
	//					|		B		|
	//					|				|
	//					+-------+
	// 
	intersect: function(csg) {
		var a = new CSG2.Node(this.clone().toPolygons());
		var b = new CSG2.Node(csg.clone().toPolygons());
		a.invert();
		b.clipTo(a);
		b.invert();
		a.clipTo(b);
		b.clipTo(a);
		a.build(b.allPolygons());
		a.invert();
		return CSG2.fromPolygons(a.allPolygons());
	},

	// Return a new CSG2 solid with solid and empty space switched. This solid is
	// not modified.
	inverse: function() {
		var csg = this.clone();
		csg.toPolygons().map(function(p) { p.flip(); });
		return csg;
	}
};

// Construct an axis-aligned solid cuboid. Optional parameters are `center` and
// `radius`, which default to `[0, 0, 0]` and `[1, 1, 1]`. The radius can be
// specified using a single number or a list of three numbers, one for each axis.
// 
// Example code:
// 
//		 var cube = CSG2.cube({
//			 center: [0, 0, 0],
//			 radius: 1
//		 });
CSG2.cube = function(options) {
	options = options || {};
	var c = new CSG2.Vector(options.center || [0, 0, 0]);
	var r = !options.radius ? [1, 1, 1] : options.radius.length ?
					 options.radius : [options.radius, options.radius, options.radius];
console.log("###",options,r);
	return CSG2.fromPolygons([
		[[0, 4, 6, 2], [-1, 0, 0]],
		[[1, 3, 7, 5], [+1, 0, 0]],
		[[0, 1, 5, 4], [0, -1, 0]],
		[[2, 6, 7, 3], [0, +1, 0]],
		[[0, 2, 3, 1], [0, 0, -1]],
		[[4, 5, 7, 6], [0, 0, +1]]
	].map(function(info) {
		return new CSG2.Polygon(info[0].map(function(i) {
			var pos = new CSG2.Vector(
				c.x + r[0] * (2 * !!(i & 1) - 1),
				c.y + r[1] * (2 * !!(i & 2) - 1),
				c.z + r[2] * (2 * !!(i & 4) - 1)
			);
			return new CSG2.Vertex(pos, new CSG2.Vector(info[1]));
		}));
	}));
};

// Construct a solid sphere. Optional parameters are `center`, `radius`,
// `slices`, and `stacks`, which default to `[0, 0, 0]`, `1`, `16`, and `8`.
// The `slices` and `stacks` parameters control the tessellation along the
// longitude and latitude directions.
// 
// Example usage:
// 
//		 var sphere = CSG2.sphere({
//			 center: [0, 0, 0],
//			 radius: 1,
//			 slices: 16,
//			 stacks: 8
//		 });
CSG2.sphere = function(options) {
	options = options || {};
	var c = new CSG2.Vector(options.center || [0, 0, 0]);
	var r = options.radius || 1;
	var polygons = [];
	if(options.iterations) { // make an icosphere
		var iterations = options.iterations|0; // int
		if(!(iterations in CSG2.sphere)) { // not cached?
			// we have to create it...
			var vertex = function(v) { v = v.unit(); return new CSG2.Vertex(v,v); };
			var addTriangle = function(a,b,c) { polygons.push(new CSG2.Polygon([vertex(a),vertex(b),vertex(c)])); };
			var bisect = function(a,b,c,iteration) {
				var ab = a.lerp(b,0.5);
				var ac = a.lerp(c,0.5);
				var bc = b.lerp(c,0.5);
				var func = iteration==iterations? addTriangle: bisect;
				func(a,ab,ac,iteration+1);
				func(b,bc,ab,iteration+1);
				func(c,ac,bc,iteration+1);
				func(ab,bc,ac,iteration+1);
			};
			var top = new CSG2.Vector(0,1,0);
			var bottom = new CSG2.Vector(0,-1,0);
			var leftFront = new CSG2.Vector(-1,0,-1);
			var leftBack = new CSG2.Vector(-1,0,1);
			var rightFront = new CSG2.Vector(1,0,-1);
			var rightBack = new CSG2.Vector(1,0,1);
			bisect(leftFront,top,rightFront,0);
			bisect(rightFront,bottom,leftFront,0);
			bisect(leftBack,top,leftFront,0);
			bisect(bottom,leftBack,leftFront,0);
			bisect(rightFront,top,rightBack,0);
			bisect(bottom,rightFront,rightBack,0);
			bisect(rightBack,top,leftBack,0);
			bisect(bottom,rightBack,leftBack,0);
			CSG2.sphere[iterations] = polygons;				
		}
		polygons = CSG2.sphere[iterations].map(function(p) {
			return new CSG2.Polygon(p.vertices.map(function(v) {
				return new CSG2.Vertex(c.plus(v.pos.times(r)),v.normal); 
			}));
		});
	} else { // make a uv-sphere
		var slices = options.slices || 16;
		var stacks = options.stacks || 8;
		var vertices;
		var vertex = function(theta, phi) {
			theta *= Math.PI * 2;
			phi *= Math.PI;
			var dir = new CSG2.Vector(
				Math.cos(theta) * Math.sin(phi),
				Math.cos(phi),
				Math.sin(theta) * Math.sin(phi)
			);
			vertices.push(new CSG2.Vertex(c.plus(dir.times(r)), dir));
		}
		for (var i = 0; i < slices; i++) {
			for (var j = 0; j < stacks; j++) {
				vertices = [];
				vertex(i / slices, j / stacks);
				if (j > 0) vertex((i + 1) / slices, j / stacks);
				if (j < stacks - 1) vertex((i + 1) / slices, (j + 1) / stacks);
				vertex(i / slices, (j + 1) / stacks);
				polygons.push(new CSG2.Polygon(vertices));
			}
		}
	}
	var csg = CSG2.fromPolygons(polygons);
	csg.bounds = {
		min: new CSG2.Vector(c.x-r,c.y-r,c.z-r),
		max: new CSG2.Vector(c.x+r,c.y+r,c.z+r)
	};
	return csg;
};

// Construct a solid cylinder. Optional parameters are `start`, `end`,
// `radius`, and `slices`, which default to `[0, -1, 0]`, `[0, 1, 0]`, `1`, and
// `16`. The `slices` parameter controls the tessellation.
// 
// Example usage:
// 
//		 var cylinder = CSG2.cylinder({
//			 start: [0, -1, 0],
//			 end: [0, 1, 0],
//			 radius: 1,
//			 slices: 16
//		 });
CSG2.cylinder = function(options) {
	options = options || {};
	var s = new CSG2.Vector(options.start || [0, -1, 0]);
	var e = new CSG2.Vector(options.end || [0, 1, 0]);
	var ray = e.minus(s);
	var r = options.radius || 1;
	var slices = options.slices || 16;
	var axisZ = ray.unit(), isY = (Math.abs(axisZ.y) > 0.5);
	var axisX = new CSG2.Vector(isY, !isY, 0).cross(axisZ).unit();
	var axisY = axisX.cross(axisZ).unit();
	var start = new CSG2.Vertex(s, axisZ.negated());
	var end = new CSG2.Vertex(e, axisZ);
	var polygons = [];
	function point(stack, slice, normalBlend) {
		var angle = slice * Math.PI * 2;
		var out = axisX.times(Math.cos(angle)).plus(axisY.times(Math.sin(angle)));
		var pos = s.plus(ray.times(stack)).plus(out.times(r));
		var normal = out.times(1 - Math.abs(normalBlend)).plus(axisZ.times(normalBlend));
		return new CSG2.Vertex(pos, normal);
	}
	for (var i = 0; i < slices; i++) {
		var t0 = i / slices, t1 = (i + 1) / slices;
		polygons.push(new CSG2.Polygon([start, point(0, t0, -1), point(0, t1, -1)]));
		polygons.push(new CSG2.Polygon([point(0, t1, 0), point(0, t0, 0), point(1, t0, 0), point(1, t1, 0)]));
		polygons.push(new CSG2.Polygon([end, point(1, t1, 1), point(1, t0, 1)]));
	}
	return CSG2.fromPolygons(polygons);
};

// # class Vector

// Represents a 3D vector.
// 
// Example usage:
// 
//		 new CSG2.Vector(1, 2, 3);
//		 new CSG2.Vector([1, 2, 3]);
//		 new CSG2.Vector({ x: 1, y: 2, z: 3 });

CSG2.Vector = function(x, y, z) {
	if (arguments.length == 3) {
		this.x = x;
		this.y = y;
		this.z = z;
	} else if ('x' in x) {
		this.x = x.x;
		this.y = x.y;
		this.z = x.z;
	} else {
		this.x = x[0];
		this.y = x[1];
		this.z = x[2];
	}
};

CSG2.Vector.prototype = {
	clone: function() {
		return new CSG2.Vector(this.x, this.y, this.z);
	},

	negated: function() {
		return new CSG2.Vector(-this.x, -this.y, -this.z);
	},

	plus: function(a) {
		return new CSG2.Vector(this.x + a.x, this.y + a.y, this.z + a.z);
	},

	minus: function(a) {
		return new CSG2.Vector(this.x - a.x, this.y - a.y, this.z - a.z);
	},

	times: function(a) {
		return new CSG2.Vector(this.x * a, this.y * a, this.z * a);
	},
	
	scale: function(s) {
		return new CSG2.Vector(this.x * s.x, this.y * s.y, this.z * s.z);
	},
	
	translate: function(t) {
		return new CSG2.Vector(this.x + t.x, this.y + t.y, this.z + t.z);
	},

	dividedBy: function(a) {
		return new CSG2.Vector(this.x / a, this.y / a, this.z / a);
	},

	dot: function(a) {
		return this.x * a.x + this.y * a.y + this.z * a.z;
	},

	lerp: function(a, t) {
		return this.plus(a.minus(this).times(t));
	},
	
	rotate: function(theta,axis_start,axis_stop) {
	  // http://local.wasp.uwa.edu.au/~pbourke/geometry/rotate/example.c
	  var	q1 = this.minus(axis_start);
		var q2 = new CSG2.Vector(0,0,0);
		var u = axis_stop.minus(axis_start).unit();
		var d = Math.sqrt(u.y*u.y + u.z*u.z);
		var cosrad = Math.cos(theta);
		var sinrad = Math.sin(theta);
    if(d != 0) {
      q2.x = q1.x;
      q2.y = q1.y * u.z / d - q1.z * u.y / d;
      q2.z = q1.y * u.y / d + q1.z * u.z / d;
    } else
      q2 = q1;
    q1.x = q2.x * d - q2.z * u.x;
    q1.y = q2.y;
    q1.z = q2.x * u.x + q2.z * d;
    q2.x = q1.x * cosrad - q1.y * sinrad;
    q2.y = q1.x * sinrad + q1.y * cosrad;
    q2.z = q1.z;
    q1.x =   q2.x * d + q2.z * u.x;
    q1.y =   q2.y;
    q1.z = - q2.x * u.x + q2.z * d;
    if (d != 0) {
      q2.x =   q1.x;
      q2.y =   q1.y * u.z / d + q1.z * u.y / d;
      q2.z = - q1.y * u.y / d + q1.z * u.z / d;
    } else
      q2 = q1;
    return q2.plus(axis_start);
	},

	distance: function(a) {
		return this.minus(a).length();
	},

	length: function() {
		var dot = this.dot(this);
		return dot? Math.sqrt(dot): 0;
	},
	
	unit: function() {
		return this.dividedBy(this.length());
	},

	cross: function(a) {
		return new CSG2.Vector(
			this.y * a.z - this.z * a.y,
			this.z * a.x - this.x * a.z,
			this.x * a.y - this.y * a.x
		);
	},
	
	toString: function() {
		return ""+this.x+" "+this.y+" "+this.z;
	}
};

// # class Vertex

// Represents a vertex of a polygon. Use your own vertex class instead of this
// one to provide additional features like texture coordinates and vertex
// colors. Custom vertex classes need to provide a `pos` property and `clone()`,
// `flip()`, and `interpolate()` methods that behave analogous to the ones
// defined by `CSG2.Vertex`. This class provides `normal` so convenience
// functions like `CSG2.sphere()` can return a smooth vertex normal, but `normal`
// is not used anywhere else.

CSG2.Vertex = function(pos, normal) {
	this.pos = new CSG2.Vector(pos);
	this.normal = new CSG2.Vector(normal);
};

CSG2.Vertex.prototype = {
	clone: function() {
		return new CSG2.Vertex(this.pos.clone(), this.normal.clone());
	},

	// Invert all orientation-specific data (e.g. vertex normal). Called when the
	// orientation of a polygon is flipped.
	flip: function() {
		this.normal = this.normal.negated();
	},

	// Create a new vertex between this vertex and `other` by linearly
	// interpolating all properties using a parameter of `t`. Subclasses should
	// override this to interpolate additional properties.
	interpolate: function(other, t) {
		return new CSG2.Vertex(
			this.pos.lerp(other.pos, t),
			this.normal.lerp(other.normal, t)
		);
	},
	
	toString: function() {
		return ""+this.pos+" "+this.normal;
	}
};

// # class Plane

// Represents a plane in 3D space.

CSG2.Plane = function(normal, w) {
	this.normal = normal;
	this.w = w;
};

// `CSG2.Plane.EPSILON` is the tolerance used by `splitPolygon()` to decide if a
// point is on the plane.
CSG2.Plane.EPSILON = 1e-5;

CSG2.Plane.fromPoints = function(a, b, c) {
	var n = b.minus(a).cross(c.minus(a)).unit();
	return new CSG2.Plane(n, n.dot(a));
};

CSG2.Plane.prototype = {
	clone: function() {
		return new CSG2.Plane(this.normal.clone(), this.w);
	},

	flip: function() {
		this.normal = this.normal.negated();
		this.w = -this.w;
	},

	// Split `polygon` by this plane if needed, then put the polygon or polygon
	// fragments in the appropriate lists. Coplanar polygons go into either
	// `coplanarFront` or `coplanarBack` depending on their orientation with
	// respect to this plane. Polygons in front or in back of this plane go into
	// either `front` or `back`.
	splitPolygon: function(polygon, coplanarFront, coplanarBack, front, back) {
		var COPLANAR = 0;
		var FRONT = 1;
		var BACK = 2;
		var SPANNING = 3;

		// Classify each point as well as the entire polygon into one of the above
		// four classes.
		var polygonType = 0;
		var types = [];
		for (var i = 0; i < polygon.vertices.length; i++) {
			var t = this.normal.dot(polygon.vertices[i].pos) - this.w;
			var type = (t < -CSG2.Plane.EPSILON) ? BACK : (t > CSG2.Plane.EPSILON) ? FRONT : COPLANAR;
			polygonType |= type;
			types.push(type);
		}

		// Put the polygon in the correct list, splitting it when necessary.
		switch (polygonType) {
			case COPLANAR:
				(this.normal.dot(polygon.plane.normal) > 0 ? coplanarFront : coplanarBack).push(polygon);
				break;
			case FRONT:
				front.push(polygon);
				break;
			case BACK:
				back.push(polygon);
				break;
			case SPANNING:
				var f = [], b = [];
				for (var i = 0; i < polygon.vertices.length; i++) {
					var j = (i + 1) % polygon.vertices.length;
					var ti = types[i], tj = types[j];
					var vi = polygon.vertices[i], vj = polygon.vertices[j];
					if (ti != BACK) f.push(vi);
					if (ti != FRONT) b.push(ti != BACK ? vi.clone() : vi);
					if ((ti | tj) == SPANNING) {
						var t = (this.w - this.normal.dot(vi.pos)) / this.normal.dot(vj.pos.minus(vi.pos));
						var v = vi.interpolate(vj, t);
						f.push(v);
						b.push(v.clone());
					}
				}
				if (f.length >= 3) front.push(new CSG2.Polygon(f, polygon.shared));
				if (b.length >= 3) back.push(new CSG2.Polygon(b, polygon.shared));
				break;
		}
	}
};

// # class Polygon

// Represents a convex polygon. The vertices used to initialize a polygon must
// be coplanar and form a convex loop. They do not have to be `CSG2.Vertex`
// instances but they must behave similarly (duck typing can be used for
// customization).
// 
// Each convex polygon has a `shared` property, which is shared between all
// polygons that are clones of each other or were split from the same polygon.
// This can be used to define per-polygon properties (such as surface color).

CSG2.Polygon = function(vertices, shared) {
	this.vertices = vertices;
	this.shared = shared;
	this.plane = CSG2.Plane.fromPoints(vertices[0].pos, vertices[1].pos, vertices[2].pos);
};

CSG2.Polygon.prototype = {
	clone: function() {
		var vertices = this.vertices.map(function(v) { return v.clone(); });
		return new CSG2.Polygon(vertices, this.shared);
	},

	flip: function() {
		this.vertices.reverse().map(function(v) { v.flip(); });
		this.plane.flip();
	}
};

// # class Node

// Holds a node in a BSP tree. A BSP tree is built from a collection of polygons
// by picking a polygon to split along. That polygon (and all other coplanar
// polygons) are added directly to that node and the other polygons are added to
// the front and/or back subtrees. This is not a leafy BSP tree since there is
// no distinction between internal and leaf nodes.

CSG2.Node = function(polygons, plane) {
	this.plane = plane; // optional
	this.front = null;
	this.back = null;
	this.polygons = [];
	if (polygons) this.build(polygons);
};

CSG2.Node.fromPolygons = function(polygons,bounds) {
	if(!polygons || !polygons.length)
		return new CSG2.Node();
	if(!bounds) bounds = CSG2.getBounds(polygons);
	var centre = bounds.min.lerp(bounds.max,0.5);
	var a = new CSG2.Plane(new CSG2.Vector(1,0,0),centre.x);
	var b = new CSG2.Plane(new CSG2.Vector(0,1,0),centre.y);
	var c = new CSG2.Plane(new CSG2.Vector(0,0,1),centre.z);
	var split = function(node,plane) {
		node.front = new CSG2.Node(null,plane.clone());
		node.back = new CSG2.Node(null,plane.clone());
	};
	var root = new CSG2.Node(null,a);
	split(root,b);
	split(root.front,c);
	split(root.back,c);
	root.build(polygons);
	return root;
}

CSG2.Node.prototype = {
	clone: function() {
		var node = new CSG2.Node();
		node.plane = this.plane && this.plane.clone();
		node.front = this.front && this.front.clone();
		node.back = this.back && this.back.clone();
		node.polygons = this.polygons.map(function(p) { return p.clone(); });
		return node;
	},

	// Convert solid space to empty space and empty space to solid space.
	invert: function() {
		for (var i = 0; i < this.polygons.length; i++) {
			this.polygons[i].flip();
		}
		this.plane.flip();
		if (this.front) this.front.invert();
		if (this.back) this.back.invert();
		var temp = this.front;
		this.front = this.back;
		this.back = temp;
	},

	// Recursively remove all polygons in `polygons` that are inside this BSP
	// tree.
	clipPolygons: function(polygons) {
		if (!this.plane) return polygons.slice();
		var front = [], back = [];
		for (var i = 0; i < polygons.length; i++) {
			this.plane.splitPolygon(polygons[i], front, back, front, back);
		}
		if (this.front) front = this.front.clipPolygons(front);
		if (this.back) back = this.back.clipPolygons(back);
		else back = [];
		return front.concat(back);
	},

	// Remove all polygons in this BSP tree that are inside the other BSP tree
	// `bsp`.
	clipTo: function(bsp) {
		this.polygons = bsp.clipPolygons(this.polygons);
		if (this.front) this.front.clipTo(bsp);
		if (this.back) this.back.clipTo(bsp);
	},

	// Return a list of all polygons in this BSP tree.
	allPolygons: function() {
		if(this.front || this.back) {
			var polygons = this.polygons.slice();
			if (this.front) polygons = polygons.concat(this.front.allPolygons());
			if (this.back) polygons = polygons.concat(this.back.allPolygons());
			return polygons;
		}
		return this.polygons;
	},
	
	getBounds: function(min,max) {
		var polygons = this.polygons;
		for(var i in polygons) {
			var vertices = polygons[i].vertices;
			for(var j in vertices) {
				var pos = vertices[j].pos;
				min.x = Math.min(min.x,pos.x);
				min.y = Math.min(min.y,pos.y);
				min.z = Math.min(min.z,pos.z);
				max.x = Math.min(max.x,pos.x);
				max.y = Math.min(max.y,pos.y);
				max.z = Math.min(max.z,pos.z);
			}
		}
		if(this.front) this.front.getBounds(min,max);
		if(this.back) this.back.getBounds(min,max);
	},

	// Build a BSP tree out of `polygons`. When called on an existing tree, the
	// new polygons are filtered down to the bottom of the tree and become new
	// nodes there. Each set of polygons is partitioned using the first polygon
	// (no heuristic is used to pick a good split).
	build: function(polygons) {
		if (!polygons.length) return;
		if (!this.plane) this.plane = polygons[0].plane.clone();
		var front = [], back = [];
		for (var i = 0; i < polygons.length; i++) {
			this.plane.splitPolygon(polygons[i], this.polygons, this.polygons, front, back);
		}
		if (front.length) {
			if (!this.front) this.front = new CSG2.Node();
			this.front.build(front);
		}
		if (back.length) {
			if (!this.back) this.back = new CSG2.Node();
			this.back.build(back);
		}
	}
};
