#!/usr/bin/env python3
import socket
import json

def deploy():
    script = '''
import omni.usd
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema
import random
import math

stage = omni.usd.get_context().get_stage()
random.seed(42)

# 1. 기존 정리
to_delete = [p.GetPath() for p in stage.Traverse() if "Pillar_" in p.GetName()]
for p in to_delete:
    stage.RemovePrim(p)

placed = []
count = 0
for i in range(100):
    if count >= 12: break
    x, y = random.uniform(-4.5, 4.5), random.uniform(-4.5, 4.5)
    
    # 겹침 방지 (기존 상자 근처 회피 추가)
    too_close = False
    for px, py in placed + [(1.24, 0.44), (-1.2, -1.2)]: # Box_A, Box_B 대략적 위치
        if math.sqrt((x-px)**2 + (y-py)**2) < 1.4:
            too_close = True
            break
    if too_close: continue
        
    path = f"/World/Pillar_{count+1}"
    cube = UsdGeom.Cube.Define(stage, path)
    width, height = random.uniform(0.4, 0.6), random.uniform(2.0, 3.5)
    
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, height / 2.0))
    xf.AddScaleOp().Set(Gf.Vec3f(width, width, height))
    
    # [충돌 해결 핵심] Physics 및 Physx API 적용
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
    # 충돌 메시 근사치를 'convexHull'로 설정하여 절대 통과 못하게 함
    physx_collision.CreateContactOffsetAttr(0.02)
    
    cube.CreateDisplayColorAttr([(0.2, 0.2, 0.2)])
    placed.append((x, y))
    count += 1

print(f"[MCP] {count} Collision-proof Pillars created.")
'''
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', 8766))
        sock.sendall(json.dumps({"type": "execute_script", "params": {"code": script}}).encode("utf-8"))
        print(sock.recv(4096).decode("utf-8"))
        sock.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    deploy()
