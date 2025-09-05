#!/usr/bin/env python3
"""
Complete ArUco Marker Generator and Combiner
整合的 ArUco 標記生成和組合工具
Package: aruco_pose_detection
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import argparse
import os
import sys

class CompleteArucoProcessor:
    def __init__(self, marker_size=512, border_size=20):
        """
        初始化處理器
        
        Args:
            marker_size (int): 單個標記的大小 (像素)
            border_size (int): 標記之間的邊框大小 (像素)
        """
        self.marker_size = marker_size
        self.border_size = border_size
        
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
        
        # 預設字典
        self.default_dictionary = '6x6_250'
    
    def generate_single_marker(self, dictionary_type, marker_id):
        """
        生成單一 ArUco 標記（內存中）
        
        Args:
            dictionary_type (str): 字典類型，如 '6x6_250'
            marker_id (int): 標記 ID
            
        Returns:
            numpy.ndarray: 標記圖像 (灰階)
        """
        if dictionary_type not in self.available_dictionaries:
            raise ValueError(f"不支援的字典類型: {dictionary_type}")
        
        try:
            # 獲取字典
            dictionary = aruco.Dictionary_get(self.available_dictionaries[dictionary_type])
            
            # 檢查 ID 是否在字典範圍內
            max_id = dictionary.bytesList.shape[0] - 1
            if marker_id > max_id:
                raise ValueError(f"標記 ID {marker_id} 超出字典 {dictionary_type} 的範圍 (0-{max_id})")
            
            # 生成標記
            marker_img = aruco.drawMarker(dictionary, marker_id, self.marker_size)
            
            print(f"✅ 生成標記: ID={marker_id}, 字典={dictionary_type}, 大小={self.marker_size}px")
            
            return marker_img
            
        except Exception as e:
            print(f"❌ 生成標記 {marker_id} 時發生錯誤: {e}")
            return None
    
    def create_combined_marker(self, layout_config, dictionary_type='6x6_250'):
        """
        創建組合標記
        
        Args:
            layout_config (dict): 佈局配置
            dictionary_type (str): ArUco 字典類型
            
        Returns:
            tuple: (組合圖像, 圖像尺寸)
        """
        layout_type = layout_config['type']
        marker_ids = layout_config['ids']
        rows = layout_config['rows']
        cols = layout_config['cols']
        
        print(f"\n🎯 創建組合標記: {layout_type}")
        print(f"   佈局: {rows}x{cols}")
        print(f"   標記IDs: {marker_ids}")
        
        # 計算總尺寸
        total_width = cols * self.marker_size + (cols + 1) * self.border_size
        total_height = rows * self.marker_size + (rows + 1) * self.border_size
        
        # 創建白色背景
        combined = np.ones((total_height, total_width), dtype=np.uint8) * 255
        
        # 放置標記
        marker_index = 0
        for row in range(rows):
            for col in range(cols):
                if marker_index >= len(marker_ids):
                    break
                    
                marker_id = marker_ids[marker_index]
                
                # 跳過空白位置 (用 255 表示)
                if marker_id == 255:
                    print(f"⬜ 跳過位置 ({row}, {col}) - 空白")
                    marker_index += 1
                    continue
                
                try:
                    # 生成標記
                    marker = self.generate_single_marker(dictionary_type, marker_id)
                    if marker is None:
                        marker_index += 1
                        continue
                    
                    # 計算位置
                    start_y = self.border_size + row * (self.marker_size + self.border_size)
                    end_y = start_y + self.marker_size
                    start_x = self.border_size + col * (self.marker_size + self.border_size)
                    end_x = start_x + self.marker_size
                    
                    # 放置標記
                    combined[start_y:end_y, start_x:end_x] = marker
                    
                    print(f"📍 已放置標記 ID {marker_id} 於位置 ({row}, {col})")
                    
                except Exception as e:
                    print(f"⚠️  放置標記 {marker_id} 時發生錯誤: {e}")
                    
                marker_index += 1
        
        print(f"📐 總尺寸: {total_width} x {total_height} 像素")
        
        return combined, (total_width, total_height)

    def create_special_diagonal_layout(self, dictionary_type='6x6_250'):
        """
        創建特殊的對角線佈局（只有左上角和右下角有標記）
        """
        print(f"\n🎯 創建對角線佈局")
        
        try:
            # 生成標記
            marker_0 = self.generate_single_marker(dictionary_type, 23)
            marker_1 = self.generate_single_marker(dictionary_type, 24)
            
            if marker_0 is None or marker_1 is None:
                return None, (0, 0)
            
            # 創建 2x2 網格
            total_width = 2 * self.marker_size + 3 * self.border_size
            total_height = 2 * self.marker_size + 3 * self.border_size
            
            # 創建白色背景
            combined = np.ones((total_height, total_width), dtype=np.uint8) * 255
            
            # 放置左上角標記 (ID 0)
            start_y = self.border_size
            end_y = start_y + self.marker_size
            start_x = self.border_size
            end_x = start_x + self.marker_size
            combined[start_y:end_y, start_x:end_x] = marker_0
            print(f"📍 已放置標記 ID 0 於左上角")
            
            # 放置右下角標記 (ID 1)
            start_y = 2 * self.border_size + self.marker_size
            end_y = start_y + self.marker_size
            start_x = 2 * self.border_size + self.marker_size
            end_x = start_x + self.marker_size
            combined[start_y:end_y, start_x:end_x] = marker_1
            print(f"📍 已放置標記 ID 1 於右下角")
            
            print(f"📐 總尺寸: {total_width} x {total_height} 像素")
            
            return combined, (total_width, total_height)
            
        except Exception as e:
            print(f"❌ 創建對角線佈局時發生錯誤: {e}")
            return None, (0, 0)

    def create_centered_horizontal_2x2_layout(self, dictionary_type='6x6_250'):
        """
        創建置中的水平雙標記佈局（2x2 總大小，但標記水平置中擺放）
        """
        print(f"\n🎯 創建置中水平雙標記佈局 (2x2 總大小)")
        
        try:
            # 生成標記
            marker_0 = self.generate_single_marker(dictionary_type, 21)
            marker_1 = self.generate_single_marker(dictionary_type, 22)
            
            if marker_0 is None or marker_1 is None:
                return None, (0, 0)
            
            # 創建與 2x2 網格相同的總尺寸
            total_width = 2 * self.marker_size + 3 * self.border_size
            total_height = 2 * self.marker_size + 3 * self.border_size
            
            # 創建白色背景
            combined = np.ones((total_height, total_width), dtype=np.uint8) * 255
            
            # 計算垂直置中位置（在中間行的位置）
            center_y = (total_height - self.marker_size) // 2
            
            # 計算兩個標記的水平位置（左右擺放但整體置中）
            # 兩個標記加上中間的間距
            markers_total_width = 2 * self.marker_size + self.border_size
            start_x_offset = (total_width - markers_total_width) // 2
            
            # 放置左側標記 (ID 0)
            start_x_left = start_x_offset
            combined[center_y:center_y + self.marker_size, 
                     start_x_left:start_x_left + self.marker_size] = marker_0
            print(f"📍 已放置標記 ID 0 於左側置中位置")
            
            # 放置右側標記 (ID 1)  
            start_x_right = start_x_offset + self.marker_size + self.border_size
            combined[center_y:center_y + self.marker_size,
                     start_x_right:start_x_right + self.marker_size] = marker_1
            print(f"📍 已放置標記 ID 1 於右側置中位置")
            
            print(f"📐 總尺寸: {total_width} x {total_height} 像素")
            
            return combined, (total_width, total_height)
            
        except Exception as e:
            print(f"❌ 創建置中水平雙標記佈局時發生錯誤: {e}")
            return None, (0, 0)

    def create_centered_2rows_3x3_layout(self, dictionary_type='6x6_250'):
        """
        創建置中的雙列佈局（3x3 總大小，但只有2列標記並置中擺放）
        """
        print(f"\n🎯 創建置中雙列佈局 (3x3 總大小)")
        
        try:
            # 生成6個標記 (ID 0-5)
            markers = []
            for i in [41,42,43,44,45,46]:
                marker = self.generate_single_marker(dictionary_type, i)
                if marker is None:
                    return None, (0, 0)
                markers.append(marker)
            
            # 創建與 3x3 網格相同的總尺寸  
            total_width = 3 * self.marker_size + 4 * self.border_size
            total_height = 3 * self.marker_size + 4 * self.border_size
            
            # 創建白色背景
            combined = np.ones((total_height, total_width), dtype=np.uint8) * 255
            
            # 計算垂直位置（上下兩列，垂直置中）
            # 兩列標記加上中間間距的總高度
            two_rows_height = 2 * self.marker_size + self.border_size
            start_y_offset = (total_height - two_rows_height) // 2
            
            # 上列的Y位置
            top_row_y = start_y_offset
            # 下列的Y位置  
            bottom_row_y = start_y_offset + self.marker_size + self.border_size
            
            # 計算水平位置（三個標記水平置中）
            three_markers_width = 3 * self.marker_size + 2 * self.border_size
            start_x_offset = (total_width - three_markers_width) // 2
            
            # 放置上列3個標記 (ID 0, 1, 2)
            for i in range(3):
                start_x = start_x_offset + i * (self.marker_size + self.border_size)
                combined[top_row_y:top_row_y + self.marker_size,
                         start_x:start_x + self.marker_size] = markers[i]
                print(f"📍 已放置標記 ID {i} 於上列位置 {i}")
            
            # 放置下列3個標記 (ID 3, 4, 5)
            for i in range(3):
                start_x = start_x_offset + i * (self.marker_size + self.border_size)
                combined[bottom_row_y:bottom_row_y + self.marker_size,
                         start_x:start_x + self.marker_size] = markers[i + 3]
                print(f"📍 已放置標記 ID {i + 3} 於下列位置 {i}")
            
            print(f"📐 總尺寸: {total_width} x {total_height} 像素")
            
            return combined, (total_width, total_height)
            
        except Exception as e:
            print(f"❌ 創建置中雙列佈局時發生錯誤: {e}")
            return None, (0, 0)

def get_layout_configs():
    """
    定義不同的佈局配置
    
    Returns:
        dict: 各種佈局配置
    """
    configs = {
        'case1_single': {
            'type': '單標記',
            'rows': 1,
            'cols': 1,
            'ids': [1],
            'description': '使用1個ArUco標記'
        },
        'case2a_horizontal': {
            'type': '水平雙標記 (2x2總大小置中)',
            'rows': 1,
            'cols': 2,
            'ids': [21, 22],
            'description': '使用2個ArUco標記 (水平置中放置，與case2b相同總大小)',
            'special': 'centered_horizontal_2x2'  # 標記需要特殊處理
        },
        'case2b_diagonal': {
            'type': '對角雙標記',
            'rows': 2,
            'cols': 2,
            'ids': [23, 255, 255, 24],  # 255表示空白位置，但會用特殊函數處理
            'description': '使用2個ArUco標記 (斜對角放置，左上與右下)',
            'special': 'diagonal'  # 標記需要特殊處理
        },
        'case3_quad': {
            'type': '四宮格',
            'rows': 2,
            'cols': 2,
            'ids': [31, 32, 33, 34],
            'description': '使用4個ArUco標記 (四宮格)'
        },
        'case4_two_rows': {
            'type': '雙列三標記 (3x3總大小置中)',
            'rows': 2,
            'cols': 3,
            'ids': [41, 42, 43, 44, 45, 46],
            'description': '使用6個ArUco標記 (分成上下兩列置中，與case5相同總大小)',
            'special': 'centered_2rows_3x3'  # 標記需要特殊處理
        },
        'case5_nine_grid': {
            'type': '九宮格',
            'rows': 3,
            'cols': 3,
            'ids': [51, 52, 53, 54, 55, 56, 57, 58, 59],
            'description': '使用9個ArUco標記 (九宮格形式)'
        }
    }
    return configs

def save_marker_info(output_dir, dictionary_type, generated_cases):
    """儲存標記資訊到檔案"""
    try:
        info_file = os.path.join(output_dir, "combined_marker_info.txt")
        
        with open(info_file, 'w', encoding='utf-8') as f:
            f.write("Combined ArUco Marker Information\n")
            f.write("=" * 40 + "\n")
            f.write(f"Dictionary: {dictionary_type}\n")
            f.write(f"Generated Cases: {len(generated_cases)}\n")
            f.write(f"Border Size: 20 pixels\n")
            f.write(f"Single Marker Size: 512x512 pixels\n\n")
            
            f.write("Generated Files:\n")
            f.write("-" * 20 + "\n")
            
            configs = get_layout_configs()
            for case_id in generated_cases:
                if case_id in configs:
                    config = configs[case_id]
                    f.write(f"{case_id}.png\n")
                    f.write(f"  - {config['description']}\n")
                    f.write(f"  - 佈局: {config['rows']}x{config['cols']}\n")
                    f.write(f"  - 標記IDs: {config['ids']}\n\n")
            
            f.write(f"Generated at: {os.path.abspath(output_dir)}/\n")
        
        print(f"📄 標記資訊已儲存至: {info_file}")
        
    except Exception as e:
        print(f"⚠️  儲存標記資訊時發生錯誤: {e}")

def main():
    parser = argparse.ArgumentParser(description='Complete ArUco Marker Generator and Combiner')
    parser.add_argument('--dict', type=str, default='6x6_250',
                       help='ArUco dictionary type (default: 6x6_250)')
    parser.add_argument('--output-dir', default='combined_markers',
                       help='輸出目錄 (預設: combined_markers)')
    parser.add_argument('--marker-size', type=int, default=512,
                       help='單個標記大小 (預設: 512)')
    parser.add_argument('--border-size', type=int, default=20,
                       help='邊框大小 (預設: 20)')
    parser.add_argument('--case', choices=['all', 'case1', 'case2a', 'case2b', 'case3', 'case4', 'case5'],
                       default='all', help='生成特定配置 (預設: all)')
    parser.add_argument('--list', action='store_true',
                       help='列出所有可用的配置')
    parser.add_argument('--list-dicts', action='store_true',
                       help='列出所有可用的字典類型')
    parser.add_argument('--preview', action='store_true',
                       help='預覽生成的標記')
    
    args = parser.parse_args()
    
    # 創建處理器
    processor = CompleteArucoProcessor(args.marker_size, args.border_size)
    
    # 列出字典類型
    if args.list_dicts:
        print("📚 可用的 ArUco 字典類型:")
        print("=" * 30)
        for dict_name in sorted(processor.available_dictionaries.keys()):
            print(f"   - {dict_name}")
        return
    
    # 列出配置
    if args.list:
        configs = get_layout_configs()
        print("\n📋 可用的配置:")
        print("=" * 80)
        for case_id, config in configs.items():
            print(f"{case_id:20} | {config['description']}")
            print(f"{'':20} | 網格: {config['rows']}x{config['cols']}, IDs: {config['ids']}")
            if config.get('special'):
                print(f"{'':20} | ⚠️  特殊處理佈局: {config['special']}")
            print("-" * 80)
        return
    
    # 驗證字典類型
    if args.dict not in processor.available_dictionaries:
        print(f"❌ 不支援的字典類型: {args.dict}")
        print(f"可用類型: {list(processor.available_dictionaries.keys())}")
        sys.exit(1)
    
    # 獲取配置
    configs = get_layout_configs()
    
    # 確定要生成的配置
    if args.case == 'all':
        cases_to_generate = list(configs.keys())
    else:
        case_map = {
            'case1': 'case1_single',
            'case2a': 'case2a_horizontal', 
            'case2b': 'case2b_diagonal',
            'case3': 'case3_quad',
            'case4': 'case4_two_rows',
            'case5': 'case5_nine_grid'
        }
        
        if args.case in case_map:
            cases_to_generate = [case_map[args.case]]
        else:
            print(f"❌ 未知的配置: {args.case}")
            sys.exit(1)
    
    print(f"\n🚀 開始生成組合 ArUco 標記...")
    print(f"📁 輸出目錄: {args.output_dir}")
    print(f"📚 字典類型: {args.dict}")
    print(f"📏 標記大小: {args.marker_size}px")
    print(f"🔲 邊框大小: {args.border_size}px")
    
    # 確保輸出目錄存在
    os.makedirs(args.output_dir, exist_ok=True)
    
    generated_images = []
    generated_cases = []
    
    for case_id in cases_to_generate:
        if case_id not in configs:
            print(f"⚠️  跳過未知配置: {case_id}")
            continue
            
        config = configs[case_id]
        output_filename = f"{case_id}.png"
        output_path = os.path.join(args.output_dir, output_filename)
        
        print(f"\n{'='*60}")
        print(f"🎯 生成 {case_id}: {config['description']}")
        
        # 根據特殊處理類型選擇函數
        if config.get('special') == 'diagonal':
            image, size = processor.create_special_diagonal_layout(args.dict)
        elif config.get('special') == 'centered_horizontal_2x2':
            image, size = processor.create_centered_horizontal_2x2_layout(args.dict)
        elif config.get('special') == 'centered_2rows_3x3':
            image, size = processor.create_centered_2rows_3x3_layout(args.dict)
        else:
            try:
                image, size = processor.create_combined_marker(config, args.dict)
            except Exception as e:
                print(f"❌ 生成 {case_id} 時發生錯誤: {e}")
                continue
        
        if image is not None:
            # 儲存圖像
            cv2.imwrite(output_path, image)
            print(f"💾 已儲存: {output_path}")
            
            generated_cases.append(case_id)
            
            if args.preview:
                generated_images.append((case_id, image, config['description']))
        else:
            print(f"❌ 無法生成 {case_id}")
    
    print(f"\n{'='*60}")
    print(f"✅ 完成！成功生成 {len(generated_cases)} 個組合標記")
    
    # 儲存資訊檔案
    if generated_cases:
        save_marker_info(args.output_dir, args.dict, generated_cases)
    
    # 預覽
    if args.preview and generated_images:
        print(f"\n👀 顯示預覽...")
        for case_id, image, description in generated_images:
            # 調整顯示大小
            display_image = image.copy()
            height, width = display_image.shape
            if max(height, width) > 800:
                scale = 800 / max(height, width)
                new_width = int(width * scale)
                new_height = int(height * scale)
                display_image = cv2.resize(display_image, (new_width, new_height))
            
            cv2.imshow(f"{case_id} - {description}", display_image)
        
        print("按任意鍵關閉預覽...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    print(f"\n🎉 所有工作完成！")
    print(f"📁 輸出檔案位於: {os.path.abspath(args.output_dir)}")

if __name__ == "__main__":
    main()