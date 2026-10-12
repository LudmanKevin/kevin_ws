from fastapi import FastAPI
import requests

app = FastAPI(title="OnRobot Eye Proxy API")
COMPUTE_BOX_IP = "192.168.10.1"

@app.get("/eye/object-detected")
def check_object_detection():
    run_url = f"http://{COMPUTE_BOX_IP}/api/dc/vision/runProcess"
    results_url = f"http://{COMPUTE_BOX_IP}/api/dc/vision/results"
    headers = {"Content-Type": "application/json"}
    
    try:
        # 1. Képelemzési folyamat indítása
        run_res = requests.post(run_url, headers=headers, timeout=5.0)
        
        # 2. Utolsó mérés eredményének lekérése
        res_check = requests.get(results_url, timeout=3.0)
        
        return {
            "run_status": run_res.status_code,
            "results_status": res_check.status_code,
            "results_text": res_check.text
        }
        
    except Exception as e:
        return {"error": str(e)}