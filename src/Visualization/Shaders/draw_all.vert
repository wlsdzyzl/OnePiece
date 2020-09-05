#version 130

    in vec3 position;

    in vec3 normal;
    in vec3 color;
    uniform mat4 MVP;
    uniform int colorType;

    uniform float materialShininess;
    uniform vec4 materialAmbient;
    uniform vec4 materialDiffuse;
    uniform vec4 materialSpecular;
    uniform vec4 lightAmbient;
    uniform vec4 lightDiffuse;
    uniform vec4 lightSpecular;


out vec4 vColor;

void main()
{

        if(colorType == 1)
        {
            vColor = vec4(-normal.xyz, 1.0);
        }
        else if(colorType == 2)
        {
            vColor = vec4(color.xyz, 1.0);
        }
        else if(colorType == 3)
        {
            // use Phong shading

            vec4 material = materialDiffuse;
	    
            vec3 eyeDir = normalize(position.xyz);
            vec4 light_dir_vec4 = vec4(eyeDir,1.0);
            vec3 light_dir = light_dir_vec4.xyz; 
            vec3 R = normalize(reflect(-normalize(light_dir), normal.xyz));
            vColor = vec4(color.xyz, 1.0);
            vec4 res = vColor * lightAmbient  * materialAmbient                                                       // Ambient
                + vColor * lightDiffuse  * material * max(dot(normal.xyz, -normalize(light_dir)), 0.0)                  // Diffuse
                + lightSpecular * materialSpecular * pow(max(dot(R, eyeDir), 0.0f), materialShininess); // Specular

            vColor = clamp(res, 0.0, 1.0);

        }
        else
        {
            // use Phong shading

            vec4 material = materialDiffuse;
	    
            vec3 eyeDir = normalize(position.xyz);
            vec4 light_dir_vec4 = vec4(eyeDir,1.0);
            vec3 light_dir = light_dir_vec4.xyz; 
            vec3 R = normalize(reflect(-normalize(light_dir), normal.xyz));
            vColor = vec4(color.xyz, 1.0);
            vec4 res = lightAmbient  * materialAmbient                                                       // Ambient
                + lightDiffuse  * material * max(dot(normal.xyz, -normalize(light_dir)), 0.0)                  // Diffuse
                + lightSpecular * materialSpecular * pow(max(dot(R, eyeDir), 0.0f), materialShininess); // Specular

            vColor = clamp(res, 0.0, 1.0);            
        }
	    gl_Position = MVP * vec4(position.xyz, 1.0);
        
}
