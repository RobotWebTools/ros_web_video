
  uniform sampler2D map;
  uniform float varianceFilter;
  uniform float colorFader;
  
  varying vec2 vUvP;
  varying vec2 colorP;
  
  varying float vari;
  varying float maskVal;
  
  
  void main() {
    
    vec4 color;
    
    float varThres = 0.001-(varianceFilter/3000.0)*0.001;
    
    if ( (vari>varThres) || (maskVal>0.5) ||(vUvP.x<0.0)|| (vUvP.x>0.5) || (vUvP.y<0.5) || (vUvP.y>1.0))
    {  
      discard;
    }
    else 
    {
      color = texture2D( map, colorP );
      
      float fader = colorFader /100.0;
      
      color.r = color.r * (1.0-fader)+ fader;
      
      color.g = color.g * (1.0-fader)+ fader;
      
      color.b = color.b * (1.0-fader)+ fader;
      
      color.a = 1.0;//smoothstep( 20000.0, -20000.0, gl_FragCoord.z / gl_FragCoord.w );
    }
    
    gl_FragColor = vec4( color.r, color.g, color.b, color.a );
    
  }
  