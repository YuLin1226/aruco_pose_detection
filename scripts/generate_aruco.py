#!/usr/bin/env python3
"""
ArUco Marker Generation Script for Gazebo Simulation
Package: aruco_pose_detection
Author: Your Name
Description: Generate ArUco markers with different sizes and IDs for Gazebo simulation
"""

import cv2
import cv2.aruco as aruco
import os
import argparse
import sys

class ArucoMarkerGenerator:
    def __init__(self):
        # 支援的字典類型
        self.available_dictionaries = {
            '4x4_50': aruco.DICT_4X4_50,
            '4x4_100': aruco.DICT_4X4_100,
            '4x4_250': aruco.DICT_4X4_250,
            '4x4_1000': aruco.DICT_4X4_1000,
            '5x5_50': aruco.DICT_5X5_50,
            '5x5_100': aruco.DICT_5X5_100,
            '5x5_250': aruco.DICT_5X5_250,
            '5x5_1000': aruco.DICT_5X5_1000,
            '6x6_50': aruco.DICT_6X6_50,
            '6x6_100': aruco.DICT_6X6_100,
            '6x6_250': aruco.DICT_6X6_250,
            '6x6_1000': aruco.DICT_6X6_1000,
            '7x7_50': aruco.DICT_7X7_50,
            '7x7_100': aruco.DICT_7X7_100,
            '7x7_250': aruco.DICT_7X7_250,
            '7x7_1000': aruco.DICT_7X7_1000,
        }
    
    def generate_single_marker(self, dictionary_type, marker_id, output_size, output_path):
        """
        生成單一 ArUco 標記
        
        Args:
            dictionary_type (str): 字典類型，如 '6x6_250'
            marker_id (int): 標記 ID
            output_size (int): 輸出圖片大小 (像素)
            output_path (str): 輸出檔案路徑
        """
        if dictionary_type not in self.available_dictionaries:
            print(f"❌ 不支援的字典類型: {dictionary_type}")
            print(f"可用的字典類型: {list(self.available_dictionaries.keys())}")
            return False
        
        try:
            # 獲取字典
            dictionary = aruco.Dictionary_get(self.available_dictionaries[dictionary_type])
            
            # 檢查 ID 是否在字典範圍內
            max_id = dictionary.bytesList.shape[0] - 1
            if marker_id > max_id:
                print(f"❌ 標記 ID {marker_id} 超出字典 {dictionary_type} 的範圍 (0-{max_id})")
                return False
            
            # 生成標記
            marker_img = aruco.drawMarker(dictionary, marker_id, output_size)
            
            # 建立輸出目錄
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            
            # 儲存圖片
            cv2.imwrite(output_path, marker_img)
            
            print(f"✅ 已生成標記: ID={marker_id}, 字典={dictionary_type}, 大小={output_size}px")
            print(f"   儲存位置: {output_path}")
            
            return True
            
        except Exception as e:
            print(f"❌ 生成標記時發生錯誤: {e}")
            return False
    
    def generate_marker_set(self, dictionary_type='6x6_250', marker_ids=None, 
                           output_size=512, output_dir='markers'):
        """
        生成一組 ArUco 標記
        
        Args:
            dictionary_type (str): 字典類型
            marker_ids (list): 要生成的標記 ID 列表
            output_size (int): 輸出圖片大小
            output_dir (str): 輸出目錄
        """
        if marker_ids is None:
            marker_ids = [0, 1, 2, 3, 4, 5]  # 預設生成 6 個標記
        
        print(f"🎯 開始生成 ArUco 標記集合")
        print(f"   字典類型: {dictionary_type}")
        print(f"   標記數量: {len(marker_ids)}")
        print(f"   圖片大小: {output_size}x{output_size} 像素")
        print(f"   輸出目錄: {output_dir}")
        print("-" * 50)
        
        success_count = 0
        
        for marker_id in marker_ids:
            output_path = os.path.join(output_dir, f"marker_{marker_id}.png")
            
            if self.generate_single_marker(dictionary_type, marker_id, output_size, output_path):
                success_count += 1
        
        print("-" * 50)
        print(f"🎉 完成！成功生成 {success_count}/{len(marker_ids)} 個標記")
        
        # 生成資訊檔案
        info_file = os.path.join(output_dir, "marker_info.txt")
        self.save_marker_info(info_file, dictionary_type, marker_ids, output_size)
        
        return success_count == len(marker_ids)
    
    def save_marker_info(self, info_file, dictionary_type, marker_ids, output_size):
        """儲存標記資訊到檔案"""
        try:
            with open(info_file, 'w') as f:
                f.write("ArUco Marker Information\n")
                f.write("=" * 30 + "\n")
                f.write(f"Dictionary: {dictionary_type}\n")
                f.write(f"Image Size: {output_size}x{output_size} pixels\n")
                f.write(f"Generated Markers: {len(marker_ids)}\n")
                f.write(f"Marker IDs: {marker_ids}\n")
                f.write("\nFile List:\n")
                for marker_id in marker_ids:
                    f.write(f"  - marker_{marker_id}.png (ID: {marker_id})\n")
                f.write(f"\nGenerated at: {os.path.dirname(os.path.abspath(info_file))}/\n")
            
            print(f"📄 標記資訊已儲存至: {info_file}")
        except Exception as e:
            print(f"⚠️  儲存標記資訊時發生錯誤: {e}")
    
    def preview_marker(self, marker_path, window_name="ArUco Marker Preview"):
        """預覽生成的標記"""
        if not os.path.exists(marker_path):
            print(f"❌ 找不到標記檔案: {marker_path}")
            return
        
        marker_img = cv2.imread(marker_path)
        if marker_img is None:
            print(f"❌ 無法讀取標記圖片: {marker_path}")
            return
        
        # 調整顯示大小
        display_size = 400
        marker_resized = cv2.resize(marker_img, (display_size, display_size))
        
        cv2.imshow(window_name, marker_resized)
        print(f"👀 正在預覽: {marker_path}")
        print("   按任意鍵關閉預覽視窗...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='ArUco Marker Generator for Gazebo Simulation')
    parser.add_argument('--dict', type=str, default='6x6_250',
                        help='ArUco dictionary type (default: 6x6_250)')
    parser.add_argument('--ids', nargs='+', type=int, default=[0, 1, 2, 3, 4, 5],
                        help='Marker IDs to generate (default: 0 1 2 3 4 5)')
    parser.add_argument('--size', type=int, default=512,
                        help='Output image size in pixels (default: 512)')
    parser.add_argument('--output', type=str, default='markers',
                        help='Output directory (default: markers)')
    parser.add_argument('--preview', action='store_true',
                        help='Preview generated markers')
    parser.add_argument('--list-dicts', action='store_true',
                        help='List available dictionary types')
    
    args = parser.parse_args()
    
    generator = ArucoMarkerGenerator()
    
    # 列出可用字典
    if args.list_dicts:
        print("📚 可用的 ArUco 字典類型:")
        for dict_name in sorted(generator.available_dictionaries.keys()):
            print(f"   - {dict_name}")
        return
    
    # 生成標記
    print("🚀 ArUco 標記生成器啟動")
    success = generator.generate_marker_set(
        dictionary_type=args.dict,
        marker_ids=args.ids,
        output_size=args.size,
        output_dir=args.output
    )
    
    if success:
        print("✨ 所有標記生成完成!")
        
        # 預覽標記
        if args.preview:
            for marker_id in args.ids:
                marker_path = os.path.join(args.output, f"marker_{marker_id}.png")
                generator.preview_marker(marker_path, f"ArUco Marker {marker_id}")
    else:
        print("💥 部分標記生成失敗，請檢查錯誤訊息")
        sys.exit(1)


if __name__ == "__main__":
    main()