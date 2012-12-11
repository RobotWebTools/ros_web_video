
  uniform sampler2D map;
  
  uniform float width;
  uniform float height;
  uniform float nearClipping, farClipping;
  
  uniform float pointSize;
  uniform float zOffset;
  
  uniform float focallength;
  
  varying vec2 vUvP;
  varying vec2 colorP;
  
  varying float vari;
  varying float maskVal;
  
  const float XtoZ = 1.11146;
  // tan( 1.0144686 / 2.0 ) * 2.0;
  const float YtoZ = 0.83359;
  // tan( 0.7898090 / 2.0 ) * 2.0;
  
  float sampleDepth(vec2 pos)
    {
      float depth;
      
      vec2 vUv = vec2( pos.x / (width*2.0), pos.y / (height*2.0)+0.5 );
      vec2 vUv2 = vec2( pos.x / (width*2.0)+0.5, pos.y / (height*2.0)+0.5 );
      
      vec4 depthColor = texture2D( map, vUv );
      
      depth = ( depthColor.r + depthColor.g + depthColor.b ) / 3.0 ;
      
      if (depth>0.99)
      {
        vec4 depthColor2 = texture2D( map, vUv2 );
        float depth2 = ( depthColor2.r + depthColor2.g + depthColor2.b ) / 3.0 ;
        depth = 0.99+depth2;
      }
      
      return depth;
    }
  
  float median(float a, float b, float c)
    {
      float r=a;
      
      if ( (a<b) && (b<c) )
      {
        r = b;
      }
      if ( (a<c) && (c<b) )
      {
        r = c;
      }
      return r;
    }
  
  float variance(float d1, float d2, float d3, float d4, float d5, float d6, float d7, float d8, float d9)
    {
      float mean = (d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8 + d9) / 9.0;
      float t1 = (d1-mean);
      float t2 = (d2-mean);
      float t3 = (d3-mean);
      float t4 = (d4-mean);
      float t5 = (d5-mean);
      float t6 = (d6-mean);
      float t7 = (d7-mean);
      float t8 = (d8-mean);
      float t9 = (d9-mean);
      float v = (t1*t1+t2*t2+t3*t3+t4*t4+t5*t5+t6*t6+t7*t7+t8*t8+t9*t9)/9.0;
      return v;
    }
  
  vec2 decodeDepth(vec2 pos)
    {
      vec2 ret;
      
      
      float depth1 = sampleDepth(vec2(position.x-1.0, position.y-1.0));
      float depth2 = sampleDepth(vec2(position.x, position.y-1.0));
      float depth3 = sampleDepth(vec2(position.x+1.0, position.y-1.0));
      float depth4 = sampleDepth(vec2(position.x-1.0, position.y));
      float depth5 = sampleDepth(vec2(position.x, position.y));
      float depth6 = sampleDepth(vec2(position.x+1.0, position.y));
      float depth7 = sampleDepth(vec2(position.x-1.0, position.y+1.0));
      float depth8 = sampleDepth(vec2(position.x, position.y+1.0));
      float depth9 = sampleDepth(vec2(position.x+1.0, position.y+1.0));
      
      float median1 = median(depth1, depth2, depth3);
      float median2 = median(depth4, depth5, depth6);
      float median3 = median(depth7, depth8, depth9);
      
      ret.x = median(median1, median2, median3);
      ret.y = variance(depth1, depth2, depth3, depth4, depth5, depth6, depth7, depth8, depth9);
      
      return ret;
      
    }
  
  
  void main() {
    
    vUvP = vec2( position.x / (width*2.0), position.y / (height*2.0)+0.5 );
    colorP = vec2( position.x / (width*2.0)+0.5 , position.y / (height*2.0)  );
    
    vec4 pos = vec4(0.0,0.0,0.0,0.0);
    vari = 0.0;
    
    if ( (vUvP.x<0.0)|| (vUvP.x>0.5) || (vUvP.y<0.5) || (vUvP.y>0.0))
    {
      
      vec2 smp = decodeDepth(vec2(position.x, position.y));
      float depth = smp.x;
      vari = smp.y;
      
      if (true)//depth>0.02)
      {
        float z = depth;
        
        pos = vec4(
          ( position.x / width - 0.5 ) * z * (1000.0/focallength),
          ( position.y / height - 0.5 ) * z * (1000.0/focallength),
          - z + zOffset / 1000.0,
          1.0);
        
        
        vec2 maskP = vec2( position.x / (width*2.0), position.y / (height*2.0)  );
        vec4 maskColor = texture2D( map, maskP );
        maskVal = ( maskColor.r + maskColor.g + maskColor.b ) / 3.0 ;
      }
      else
      {
        maskVal = 1.0;
      }
      
    }
    
    gl_PointSize = pointSize;
    gl_Position = projectionMatrix * modelViewMatrix * pos;
    
  }
  