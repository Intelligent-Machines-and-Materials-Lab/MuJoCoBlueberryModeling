import glfw
if not glfw.init():
    print("GLFW init failed")
else:
    window = glfw.create_window(640, 480, "Test", None, None)
    if not window:
        print("Window creation failed")
    else:
        print("Window created successfully")
        glfw.destroy_window(window)
    glfw.terminate()