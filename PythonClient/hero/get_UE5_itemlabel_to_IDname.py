# Paste this into UE5 editor 
import unreal, csv

actors = unreal.EditorLevelLibrary.get_all_level_actors()
with open('/home/sgarimella34/multi-robot-coordination/HERCULES/csv_data/ue_label_vs_name.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['actor_label', 'get_name'])
    for actor in actors:
        w.writerow([actor.get_actor_label(), actor.get_name()])
